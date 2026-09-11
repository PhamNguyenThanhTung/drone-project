#!/usr/bin/env python3
"""
Clean, High-Performance YOLOv8 + ByteTrack Single-Person Vision Tracking Node.

Pipeline:
    sensor_msgs/Image -> cv_bridge -> YOLOv8n.track(ByteTrack)
    -> Person Detection & Aspect Ratio / Area Filtering
    -> Auto-Lock Target -> Smooth Bounding Box (EMA)
    -> Publish /tracking/error (geometry_msgs/Point)
    -> Render Clean Live HUD -> /tracking/debug_image
"""

import time
import cv2
import torch
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO

STATE_TRACKING = 'TRACKING'
STATE_LOST = 'LOST'


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('error_topic', '/tracking/error')
        self.declare_parameter('debug_image_topic', '/tracking/debug_image')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('img_width', 416)
        self.declare_parameter('img_height', 416)
        self.declare_parameter('infer_native', True)
        self.declare_parameter('infer_imgsz', 0)
        self.declare_parameter('conf', 0.45)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('classes', [0])  # Class 0 = person in COCO
        self.declare_parameter('min_box_area_ratio', 0.0003)
        # Keep very large, near-camera boxes available so the arbiter can use
        # its BACKING_UP_TO_RECOVER grace window instead of losing the target
        # at the bottom edge prematurely.
        self.declare_parameter('max_box_area_ratio', 0.85)
        # A person seen from a 37 deg down-pitched camera is frequently WIDER
        # than tall (leaning, arms out, walking across the frame). The old
        # 1.10 floor threw those detections away: the person was visible on
        # screen yet had no box, so clicks did nothing and the lock dropped
        # out every time the pose changed.
        self.declare_parameter('min_aspect_ratio', 0.70)
        self.declare_parameter('max_aspect_ratio', 4.80)
        self.declare_parameter('bottom_margin_ratio', 0.0)
        self.declare_parameter('show_debug_image', True)
        self.declare_parameter('target_timeout', 4.0)
        # ByteTrack hands out a fresh track id whenever a person is missed for
        # a few frames (very common at CPU frame rates). Without re-binding,
        # a manual lock died the instant the id changed. These two knobs bound
        # how long and how far the lock may follow such an id switch.
        self.declare_parameter('lock_reacquire_s', 5.0)
        self.declare_parameter('reacquire_min_iou', 0.15)
        self.declare_parameter('log_period', 1.5)
        self.declare_parameter('max_frame_rate', 0.0)

        gp = self.get_parameter
        self.image_topic = gp('image_topic').value
        self.W = int(gp('img_width').value)
        self.H = int(gp('img_height').value)
        self.infer_native = bool(gp('infer_native').value)
        self.infer_imgsz = int(gp('infer_imgsz').value)
        self.conf = float(gp('conf').value)
        self.iou = float(gp('iou').value)
        self.classes = [int(c) for c in gp('classes').value]
        self.min_box_area_ratio = float(gp('min_box_area_ratio').value)
        self.max_box_area_ratio = float(gp('max_box_area_ratio').value)
        self.min_aspect_ratio = float(gp('min_aspect_ratio').value)
        self.max_aspect_ratio = float(gp('max_aspect_ratio').value)
        self.bottom_margin_ratio = float(gp('bottom_margin_ratio').value)
        self.device = gp('device').value
        if isinstance(self.device, str) and self.device.lower().startswith('cuda'):
            if not torch.cuda.is_available():
                raise RuntimeError(
                    'YOLO requested CUDA (%s), but PyTorch cannot access an NVIDIA GPU. '
                    'Repair the Windows NVIDIA driver/WSL GPU passthrough.' % self.device
                )
            self.get_logger().info(
                'CUDA device available: %s (%s)' % (
                    self.device, torch.cuda.get_device_name(0)
                )
            )
        self.tracker = gp('tracker').value
        self.show_debug_image = bool(gp('show_debug_image').value)
        self.target_timeout = float(gp('target_timeout').value)
        self.lock_reacquire_s = float(gp('lock_reacquire_s').value)
        self.reacquire_min_iou = float(gp('reacquire_min_iou').value)
        self.log_period = float(gp('log_period').value)
        self.max_frame_rate = float(gp('max_frame_rate').value)
        self.last_inference_monotonic = 0.0

        self.bridge = CvBridge()
        self.get_logger().info(
            'Loading YOLO model [%s] on device [%s]' % (gp('model_path').value, self.device))
        self.model = YOLO(gp('model_path').value)

        self.pub_error = self.create_publisher(Point, gp('error_topic').value, 10)
        self.pub_select = self.create_publisher(Int32, '/tracking/select_target', 10)
        self.pub_debug = None
        if self.show_debug_image:
            self.pub_debug = self.create_publisher(Image, gp('debug_image_topic').value, 2)

        self.sub_select = self.create_subscription(
            Int32, '/tracking/select_target', self.on_select_target, 10)
        self.sub_click = self.create_subscription(
            Point, '/tracking/click_point', self.on_click_point, 10)

        # Camera data is perishable.  A reliable queue can make inference
        # process old frames after a brief GPU/ROS scheduling stall, producing
        # delayed errors that destabilize the flight controller.  Keep only
        # the newest frame and let the camera stream drop stale samples.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, self.image_topic, self.on_image, qos)

        # target state
        self.state = STATE_LOST
        self.target_id = None
        self.manual_target_id = None
        self.last_seen = 0.0
        self.logged_source_size = None
        self.current_cands = []
        self.all_persons = []
        self.smooth_box = None
        self.lock_lost_since = None
        self.n_id_remaps = 0
        self.debug_w = self.W
        self.debug_h = self.H
        self.sx = 1.0
        self.sy = 1.0

        # metrics
        self.n_frames = 0
        self.n_detected = 0
        self.sum_latency = 0.0
        self.max_latency = 0.0
        self.n_track_switch = 0
        self.n_lost_events = 0
        self.last_log = time.time()

        self.get_logger().info(
            'yolo_detector_node ready: image_topic=%s error_frame=%dx%d '
            'infer_native=%s tracker=%s device=%s conf=%.2f'
            % (self.image_topic, self.W, self.H, self.infer_native,
               self.tracker, self.device, self.conf))

    # ------------------------------------------------------------------
    def on_image(self, msg):
        now = time.monotonic()
        if (self.max_frame_rate > 0.0 and self.last_inference_monotonic > 0.0
                and now - self.last_inference_monotonic < 1.0 / self.max_frame_rate):
            return
        self.last_inference_monotonic = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('cv_bridge conversion failed: %s' % exc)
            return

        src_h, src_w = frame.shape[:2]
        if self.logged_source_size != (src_w, src_h):
            self.logged_source_size = (src_w, src_h)
            self.get_logger().info(
                'Original image: %dx%d | Inference image: %s | Error frame: %dx%d'
                % (src_w, src_h,
                   ('%dx%d (native)' % (src_w, src_h)) if self.infer_native
                   else ('%dx%d' % (self.W, self.H)),
                   self.W, self.H))

        if self.infer_native:
            infer = frame
        elif (src_w, src_h) != (self.W, self.H):
            infer = cv2.resize(frame, (self.W, self.H))
        else:
            infer = frame

        ih, iw = infer.shape[:2]
        self.debug_w, self.debug_h = iw, ih
        self.sx = self.W / float(iw)
        self.sy = self.H / float(ih)
        imgsz = self.infer_imgsz if self.infer_imgsz > 0 else max(iw, ih)

        t0 = time.time()
        results = self.model.track(
            infer,
            persist=True,
            tracker=self.tracker,
            classes=self.classes,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            imgsz=imgsz,
            verbose=False,
        )
        latency = time.time() - t0

        self.n_frames += 1
        self.sum_latency += latency
        self.max_latency = max(self.max_latency, latency)

        # 1. Parse & filter detected persons
        cands = self.extract_candidates(results, infer)
        self.current_cands = cands

        # 2. Select target person (Auto-lock single person or user manual selection)
        target = self.select_target(cands)
        now = time.time()

        if target is not None:
            self.n_detected += 1
            self.last_seen = now
            tid, x1, y1, x2, y2, cf, area = target

            # Apply EMA smoothing on bounding box
            if self.smooth_box is not None and self.target_id == tid:
                alpha = 0.60
                sx1 = alpha * x1 + (1.0 - alpha) * self.smooth_box[0]
                sy1 = alpha * y1 + (1.0 - alpha) * self.smooth_box[1]
                sx2 = alpha * x2 + (1.0 - alpha) * self.smooth_box[2]
                sy2 = alpha * y2 + (1.0 - alpha) * self.smooth_box[3]
                self.smooth_box = (sx1, sy1, sx2, sy2)
            else:
                self.smooth_box = (x1, y1, x2, y2)
                if self.target_id is not None and self.target_id != tid:
                    self.n_track_switch += 1

            self.target_id = tid
            if self.state != STATE_TRACKING:
                self.get_logger().info('state LOST -> TRACKING (target_id=%s)' % self.target_id)
            self.state = STATE_TRACKING

            target_tuple = (self.target_id, self.smooth_box[0], self.smooth_box[1],
                            self.smooth_box[2], self.smooth_box[3], cf)
            self.publish_error(target_tuple)
        else:
            if self.state == STATE_TRACKING and (now - self.last_seen) > self.target_timeout:
                self.get_logger().warn(
                    'target lost for %.2fs -> LOST (was target_id=%s)'
                    % (now - self.last_seen, self.target_id))
                self.state = STATE_LOST
                self.target_id = None
                self.smooth_box = None
                self.n_lost_events += 1

        if self.pub_debug is not None:
            self.publish_debug(infer, target)

        self.log_metrics()

    def on_select_target(self, msg):
        req_id = int(msg.data)
        if req_id < 0:
            self.get_logger().info('[YOLO] Target selection CLEARED -> Standby')
            self.manual_target_id = -1
            self.target_id = None
            self.smooth_box = None
            self.lock_lost_since = None
            self.state = STATE_LOST
        elif req_id == self.manual_target_id:
            # Echo of our own re-acquire publish; keep the existing lock state.
            return
        else:
            self.get_logger().info(f'[YOLO] Target LOCKED to Person ID: {req_id}')
            self.manual_target_id = req_id
            self.target_id = req_id
            self.lock_lost_since = None
            # Drop the old EMA anchor: keeping it blended the previous
            # person's box into the newly selected one for several frames.
            self.smooth_box = None

    def on_click_point(self, msg):
        """Resolve a HUD pixel click against every detected person."""
        # Deliberately uses all_persons, not current_cands: a pilot pointing at
        # somebody on screen must be able to lock them even if the box shape
        # failed the automatic-selection filters.
        pool = self.all_persons or self.current_cands
        if not pool:
            self.get_logger().info('[YOLO] HUD click ignored: no person boxes available')
            return
        x = float(msg.x)
        y = float(msg.y)
        candidates = []
        for cand in pool:
            tid, x1, y1, x2, y2, _cf, _area = cand
            inside = x1 <= x <= x2 and y1 <= y <= y2
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            distance = (x - cx) ** 2 + (y - cy) ** 2
            candidates.append((0 if inside else 1, distance, tid))
        _inside_rank, _distance, tid = min(candidates)
        if _inside_rank != 0 and _distance > 120.0 ** 2:
            self.get_logger().info('[YOLO] HUD click ignored: outside tracked boxes')
            return
        out = Int32()
        out.data = int(tid)
        self.pub_select.publish(out)
        self.get_logger().info(f'[YOLO] HUD click resolved to Person ID: {tid}')

    # ------------------------------------------------------------------
    def extract_candidates(self, results, frame):
        if not results:
            self.all_persons = []
            return []
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            self.all_persons = []
            return []

        ids = boxes.id.int().tolist() if boxes.id is not None else list(range(len(boxes)))
        xyxy = boxes.xyxy.tolist()
        confs = boxes.conf.tolist()

        ih, iw = frame.shape[:2]
        frame_area = float(iw * ih)
        min_area = frame_area * self.min_box_area_ratio
        max_area = frame_area * self.max_box_area_ratio
        bottom_limit = ih * (1.0 - self.bottom_margin_ratio)

        cands = []
        # Every class-0 detection, geometry filters NOT applied. A HUD click is
        # an explicit human decision: if YOLO saw a person where the pilot
        # clicked, the lock must be allowed even when the box shape would be
        # rejected for automatic selection.
        all_persons = []
        # The person currently being tracked also survives the shape gates: a
        # walker who turns, leans or swings an arm briefly produces a wide or
        # very large box, and dropping those frames was enough to break the
        # track id and lose the lock entirely.
        if self.manual_target_id is not None and self.manual_target_id >= 0:
            keep_id = self.manual_target_id
        else:
            keep_id = self.target_id
        for tid, box, cf in zip(ids, xyxy, confs):
            x1, y1, x2, y2 = box
            x1 = max(0.0, min(float(iw - 1), float(x1)))
            y1 = max(0.0, min(float(ih - 1), float(y1)))
            x2 = max(0.0, min(float(iw - 1), float(x2)))
            y2 = max(0.0, min(float(ih - 1), float(y2)))

            w = max(1.0, float(x2 - x1))
            h = max(1.0, float(y2 - y1))
            area = w * h
            aspect_ratio = h / w
            entry = (int(tid), x1, y1, x2, y2, float(cf), float(area))
            all_persons.append(entry)

            is_locked = keep_id is not None and int(tid) == int(keep_id)
            if y2 > bottom_limit:
                continue
            if not is_locked and (area < min_area or area > max_area):
                continue
            if not is_locked and (aspect_ratio < self.min_aspect_ratio
                                  or aspect_ratio > self.max_aspect_ratio):
                continue

            cands.append(entry)

        self.all_persons = all_persons

        return cands

    def select_target(self, cands):
        if not cands:
            return None

        if self.manual_target_id == -1:
            # Standby mode: do not auto-track
            return None

        if self.manual_target_id is not None and self.manual_target_id >= 0:
            matched = [c for c in cands if c[0] == self.manual_target_id]
            if matched:
                self.lock_lost_since = None
                return matched[0]
            # The locked id vanished. Before giving up, check whether one of
            # the current boxes is plainly the same person under a new
            # ByteTrack id (overlapping the last known box).
            return self._reacquire_lock(cands)

        # Auto-track mode. Stay on the person already being tracked as long as
        # they are still detected: picking max(area*conf) every frame made the
        # target id flip between people (and between frames) whenever their box
        # sizes were close, which the HUD showed as a constantly changing ID.
        if self.target_id is not None:
            same = [c for c in cands if c[0] == self.target_id]
            if same:
                return same[0]
        return max(cands, key=lambda c: c[6] * c[5])

    @staticmethod
    def _iou(box_a, box_b):
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0.0:
            return 0.0
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - inter
        return inter / union if union > 0.0 else 0.0

    def _reacquire_lock(self, cands):
        """Re-bind a manual lock to the same person after a track-id switch using Hybrid IoU & Centroid Proximity."""
        if self.smooth_box is None:
            return None
        now = time.time()
        if self.lock_lost_since is None:
            self.lock_lost_since = now
        if now - self.lock_lost_since > self.lock_reacquire_s:
            return None

        ref_x1, ref_y1, ref_x2, ref_y2 = self.smooth_box
        ref_cx, ref_cy = (ref_x1 + ref_x2) / 2.0, (ref_y1 + ref_y2) / 2.0
        ref_w = max(1.0, ref_x2 - ref_x1)
        ref_h = max(1.0, ref_y2 - ref_y1)

        best, best_score, best_iou = None, -1.0, 0.0
        for cand in cands:
            iou = self._iou(self.smooth_box, cand[1:5])
            c_x1, c_y1, c_x2, c_y2 = cand[1:5]
            c_cx, c_cy = (c_x1 + c_x2) / 2.0, (c_y1 + c_y2) / 2.0

            # Normalized Euclidean distance relative to box dimensions
            dist_x = abs(c_cx - ref_cx) / max(ref_w, 30.0)
            dist_y = abs(c_cy - ref_cy) / max(ref_h, 30.0)
            dist_norm = (dist_x ** 2 + dist_y ** 2) ** 0.5

            proximity_score = max(0.0, 1.0 - dist_norm / 2.0)
            hybrid_score = 0.60 * iou + 0.40 * proximity_score

            # Accept if standard IoU is met OR spatial proximity is very close despite camera shift
            is_valid = (iou >= self.reacquire_min_iou) or (dist_norm < 1.2 and proximity_score >= 0.45)
            if is_valid and hybrid_score > best_score:
                best = cand
                best_score = hybrid_score
                best_iou = iou

        if best is None:
            return None

        old_id = self.manual_target_id
        self.manual_target_id = int(best[0])
        self.lock_lost_since = None
        self.n_id_remaps += 1
        self.get_logger().info(
            '[YOLO] lock re-acquired: track id %s -> %s (IoU %.2f, Hybrid %.2f)'
            % (old_id, self.manual_target_id, best_iou, best_score))
        # Keep the arbiter and the HUD banner on the same id.
        out = Int32()
        out.data = int(self.manual_target_id)
        self.pub_select.publish(out)
        return best

    # ------------------------------------------------------------------
    def publish_error(self, target):
        _tid, x1, y1, x2, y2, _cf = target
        x1, x2 = x1 * self.sx, x2 * self.sx
        y1, y2 = y1 * self.sy, y2 * self.sy
        cx, cy = self.W / 2.0, self.H / 2.0
        tx, ty = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        msg = Point()
        msg.x = tx - cx
        msg.y = ty - cy
        msg.z = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        self.pub_error.publish(msg)

    def publish_debug(self, frame, target):
        img = frame.copy()
        ih, iw = img.shape[:2]

        # Draw 50% active tracking safe zone box
        zx1, zx2 = int(0.25 * iw), int(0.75 * iw)
        zy1, zy2 = int(0.25 * ih), int(0.75 * ih)
        z_color = (0, 255, 255)
        bracket_len = 25
        cv2.line(img, (zx1, zy1), (zx1 + bracket_len, zy1), z_color, 1)
        cv2.line(img, (zx1, zy1), (zx1, zy1 + bracket_len), z_color, 1)
        cv2.line(img, (zx2, zy1), (zx2 - bracket_len, zy1), z_color, 1)
        cv2.line(img, (zx2, zy1), (zx2, zy1 + bracket_len), z_color, 1)
        cv2.line(img, (zx1, zy2), (zx1 + bracket_len, zy2), z_color, 1)
        cv2.line(img, (zx1, zy2), (zx1, zy2 - bracket_len), z_color, 1)
        cv2.line(img, (zx2, zy2), (zx2 - bracket_len, zy2), z_color, 1)
        cv2.line(img, (zx2, zy2), (zx2, zy2 - bracket_len), z_color, 1)
        cv2.putText(
            img, "50% SAFE ZONE", (zx1 + 5, zy1 + 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

        # Persons that failed the automatic-selection filters are still drawn,
        # thin and grey, because they ARE clickable. Previously they were
        # invisible, which made the HUD look like the detector had missed an
        # obvious person.
        cand_ids = {c[0] for c in self.current_cands}
        for tid, x1, y1, x2, y2, cf, _area in self.all_persons:
            if tid in cand_ids:
                continue
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (150, 150, 150), 1)
            cv2.putText(
                img, "person (click)", (int(x1) + 3, max(12, int(y1) - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 150, 150), 1, cv2.LINE_AA)

        # Draw detected target box
        for cand in self.current_cands:
            tid, x1, y1, x2, y2, cf, area = cand
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            is_target = target is not None and tid == target[0]
            color = (0, 255, 0) if is_target else (255, 180, 0)

            # Bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3 if is_target else 2)

            # Label text, deliberately WITHOUT the track id: ByteTrack hands
            # out a fresh number every few frames at CPU rates, so a numeric
            # label read as "tracking keeps jumping between people and then
            # losing them". The id stays internal only.
            lock_mark = ' [LOCK]' if (
                is_target and self.manual_target_id is not None
                and self.manual_target_id >= 0) else ''
            label = "PERSON%s" % (lock_mark if is_target else '')
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            label_y = max(th + 4, y1)
            cv2.rectangle(img, (x1, label_y - th - 4), (x1 + tw + 6, label_y + 2), color, -1)
            cv2.putText(
                img, label, (x1 + 3, label_y - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

        # Tracking status text. Same rule as the box labels: no track id on
        # screen, the number churns with every ByteTrack re-assignment.
        if self.state == STATE_TRACKING:

            lock_kind = 'LOCK' if (
                self.manual_target_id is not None
                and self.manual_target_id >= 0) else 'AUTO'
            status_text = 'TRACKING PERSON [%s]' % lock_kind
            badge_color = (0, 200, 0)
        elif self.current_cands or self.all_persons:
            status_text = "CLICK A PERSON TO LOCK"
            badge_color = (0, 200, 255)
        else:
            status_text = "SEARCHING PERSON..."
            badge_color = (0, 140, 255)

        # Status banner. Kept out of the top-left corner: the HUD paints its
        # minimap there on top of this image and hid the banner completely.
        (sw, sh), _ = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        bx = int(iw * 0.33)
        cv2.rectangle(img, (bx, 10), (bx + sw + 10, 20 + sh + 6), (30, 30, 30), -1)
        cv2.rectangle(img, (bx, 10), (bx + sw + 10, 20 + sh + 6), badge_color, 2)
        cv2.putText(
            img, status_text, (bx + 5, 16 + sh),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, badge_color, 1, cv2.LINE_AA)

        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

    def log_metrics(self):
        now = time.time()
        if now - self.last_log < self.log_period:
            return
        self.last_log = now
        rate = 100.0 * self.n_detected / self.n_frames if self.n_frames else 0.0
        self.get_logger().info(
            'state=%s target_id=%s det_rate=%.1f%% mean_latency=%.3fs '
            'switches=%d id_remaps=%d lost_events=%d'
            % (self.state, self.target_id, rate,
               self.sum_latency / max(1, self.n_frames),
               self.n_track_switch, self.n_id_remaps, self.n_lost_events))


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
