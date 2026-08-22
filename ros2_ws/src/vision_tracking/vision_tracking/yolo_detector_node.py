#!/usr/bin/env python3
"""YOLOv8 + ByteTrack person detector for Phase 2 gimbal tracking.

Pipeline:
    sensor_msgs/Image -> cv_bridge -> YOLOv8n.track(ByteTrack)
    -> class 0 (person) -> track_id -> target selection -> bbox center
    -> scale into the error reference frame -> geometry_msgs/Point on /tracking/error

Error convention (in the error reference frame, img_width x img_height = 416x416,
which is the frame the Phase 2 pass criteria are defined on):
    x = target_x - center_x   (positive: target right of image center)
    y = target_y - center_y   (positive: target below image center)
    z = bounding box area in pixels (diagnostic only, never a PID input)

Inference runs on the *native* camera frame by default (infer_native=True).
Measured on this world with the 640x480 / 10 Hz Phase 1 camera: the walking
actor is only ~26 px tall, and pre-resizing to 416x416 dropped the ByteTrack
availability from 61% to 35%.  The published error is still expressed in the
416x416 frame so the criteria stay comparable.
"""

import time

import cv2
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
        self.declare_parameter('conf', 0.50)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('classes', [0])
        self.declare_parameter('min_box_area_ratio', 0.0005)
        self.declare_parameter('max_box_area_ratio', 0.25)
        self.declare_parameter('min_aspect_ratio', 1.20)
        self.declare_parameter('max_aspect_ratio', 4.50)
        self.declare_parameter('bottom_margin_ratio', 0.05)
        self.declare_parameter('show_debug_image', False)
        self.declare_parameter('target_timeout', 4.0)
        self.declare_parameter('log_period', 1.0)
        self.declare_parameter('default_target_id', -1)

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
        self.tracker = gp('tracker').value
        self.show_debug_image = bool(gp('show_debug_image').value)
        self.target_timeout = float(gp('target_timeout').value)
        self.log_period = float(gp('log_period').value)
        default_id = int(gp('default_target_id').value)
        self.selected_target_id = default_id if default_id >= 0 else None

        self.bridge = CvBridge()
        self.get_logger().info(
            'Loading YOLO model [%s] on device [%s]' % (gp('model_path').value, self.device))
        self.model = YOLO(gp('model_path').value)

        self.pub_error = self.create_publisher(Point, gp('error_topic').value, 10)
        self.pub_debug = None
        if self.show_debug_image:
            self.pub_debug = self.create_publisher(Image, gp('debug_image_topic').value, 2)

        self.sub_select = self.create_subscription(Int32, '/tracking/select_target', self.on_select_target, 10)
        self.sub_click = self.create_subscription(Point, '/tracking/click_point', self.on_click_point, 10)

        qos = QoSProfile(depth=2,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, self.image_topic, self.on_image, qos)

        # target state
        self.state = STATE_LOST
        self.target_id = None
        self.last_seen = 0.0
        self.logged_source_size = None
        self.current_cands = []
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
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001 - report and keep running
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
        # box coords are produced in the inference frame; scale them into the
        # error reference frame so the published error stays in 416x416 units
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

        target = self.select_target(results)
        now = time.time()

        if target is not None:
            self.n_detected += 1
            self.last_seen = now
            if self.state != STATE_TRACKING:
                self.get_logger().info('state LOST -> TRACKING (track_id=%s)' % self.target_id)
            self.state = STATE_TRACKING
            self.publish_error(target)
        else:
            if self.state == STATE_TRACKING and (now - self.last_seen) > self.target_timeout:
                self.get_logger().warn(
                    'target lost for %.2fs -> LOST (was track_id=%s)'
                    % (now - self.last_seen, self.target_id))
                self.state = STATE_LOST
                self.target_id = None
                self.n_lost_events += 1

        if self.pub_debug is not None:
            self.publish_debug(infer, results, target)

        self.log_metrics()

    def on_select_target(self, msg):
        req_id = int(msg.data)
        if req_id < 0:
            self.get_logger().info('[YOLO] Target selection CLEARED -> Drone STANDBY / HOVER')
            self.selected_target_id = None
            self.target_id = None
            self.state = STATE_LOST
        else:
            self.get_logger().info(f'[YOLO] Target LOCKED to Person ID: {req_id}')
            self.selected_target_id = req_id
            self.target_id = req_id

    def on_click_point(self, msg):
        cx = float(msg.x)
        cy = float(msg.y)
        if not self.current_cands:
            return
        matched_id = None
        min_d2 = 999999.0
        for tid, x1, y1, x2, y2, cf, area in self.current_cands:
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                matched_id = tid
                break
            bx, by = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            d2 = (bx - cx)**2 + (by - cy)**2
            if d2 < min_d2 and d2 < (120.0**2):
                min_d2 = d2
                matched_id = tid

        if matched_id is not None:
            self.get_logger().info(f'[YOLO] User clicked on Person ID: {matched_id} -> LOCKED')
            self.selected_target_id = matched_id
            self.target_id = matched_id
        else:
            self.get_logger().info('[YOLO] Clicked empty area -> CLEARED (STANDBY)')
            self.selected_target_id = None
            self.target_id = None
            self.state = STATE_LOST

    # ------------------------------------------------------------------
    def select_target(self, results):
        """Return (track_id, xmin, ymin, xmax, ymax, conf) for the chosen person."""
        self.current_cands = []
        if not results:
            return None
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None

        ids = boxes.id.int().tolist() if boxes.id is not None else list(range(len(boxes)))
        xyxy = boxes.xyxy.tolist()
        confs = boxes.conf.tolist()
        frame_area = float(max(1, getattr(results[0], 'orig_shape', (1, 1))[0]
                               * getattr(results[0], 'orig_shape', (1, 1))[1]))
        frame_h = float(getattr(results[0], 'orig_shape', (1, 1))[0])
        min_area = frame_area * self.min_box_area_ratio
        max_area = frame_area * self.max_box_area_ratio
        bottom_limit = frame_h * (1.0 - self.bottom_margin_ratio)

        cands = []
        for tid, box, cf in zip(ids, xyxy, confs):
            x1, y1, x2, y2 = box
            w = max(1.0, float(x2 - x1))
            h = max(1.0, float(y2 - y1))
            area = w * h
            aspect_ratio = h / w

            if y2 > bottom_limit:
                continue
            if area < min_area or area > max_area:
                continue
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue

            cands.append((tid, x1, y1, x2, y2, cf, area))

        self.current_cands = cands
        if not cands:
            return None

        # NẾU NGƯỜI DÙNG ĐÃ CHỌN 1 ID CỤ THỂ (hoặc qua Click / Phím 1-9):
        if self.selected_target_id is not None:
            matched = [c for c in cands if c[0] == self.selected_target_id]
            if matched:
                best = matched[0]
            else:
                # Target đã chọn tạm thời bị khuất hoặc chưa thấy
                return None
        else:
            # NẾU CHƯA CHỌN AI: Drone đứng yên hover, không gửi lệnh tracking
            return None

        tid, x1, y1, x2, y2, cf, area = best

        # Apply EMA (Exponential Moving Average) smoothing on bounding box
        if hasattr(self, 'smooth_box') and self.smooth_box is not None:
            alpha = 0.60
            sx1 = alpha * x1 + (1.0 - alpha) * self.smooth_box[0]
            sy1 = alpha * y1 + (1.0 - alpha) * self.smooth_box[1]
            sx2 = alpha * x2 + (1.0 - alpha) * self.smooth_box[2]
            sy2 = alpha * y2 + (1.0 - alpha) * self.smooth_box[3]
            self.smooth_box = (sx1, sy1, sx2, sy2)
        else:
            self.smooth_box = (x1, y1, x2, y2)

        cur_time = time.time()
        new_pos = ((self.smooth_box[0] + self.smooth_box[2]) / 2.0,
                   (self.smooth_box[1] + self.smooth_box[3]) / 2.0)

        if hasattr(self, 'last_target_pos') and hasattr(self, 'last_target_time'):
            dt_pos = cur_time - self.last_target_time
            if 0.01 < dt_pos < 1.0:
                raw_vx = (new_pos[0] - self.last_target_pos[0]) / dt_pos
                raw_vy = (new_pos[1] - self.last_target_pos[1]) / dt_pos
                old_vx, old_vy = getattr(self, 'target_vel', (0.0, 0.0))
                self.target_vel = (0.4 * raw_vx + 0.6 * old_vx, 0.4 * raw_vy + 0.6 * old_vy)

        self.last_target_pos = new_pos
        self.last_target_time = cur_time
        self.target_id = tid

        return (self.target_id, self.smooth_box[0], self.smooth_box[1],
                self.smooth_box[2], self.smooth_box[3], cf)

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

    def publish_debug(self, frame, results, target):
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
        cv2.putText(img, "50% SAFE ZONE", (zx1 + 5, zy1 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

        # Draw all valid detected boxes with confidence and track ID
        for cand in self.current_cands:
            tid, x1, y1, x2, y2, cf, area = cand
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            is_locked = (self.selected_target_id is not None and tid == self.selected_target_id)
            color = (0, 255, 0) if is_locked else (255, 180, 0)  # Green for locked target, Cyan for candidate

            # Bounding box
            thickness = 3 if is_locked else 2
            cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

            # Label text
            if is_locked:
                label = f"LOCKED ID: {tid} ({cf*100:.0f}%)"
            else:
                label = f"[ID: {tid}] Click/Press {tid} ({cf*100:.0f}%)"

            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            label_y = max(th + 4, y1)
            cv2.rectangle(img, (x1, label_y - th - 4), (x1 + tw + 6, label_y + 2), color, -1)
            cv2.putText(img, label, (x1 + 3, label_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

        # Top-left status banner
        if self.selected_target_id is not None:
            if target is not None:
                status_text = f"TRACKING TARGET [ID: {self.selected_target_id}]"
                badge_color = (0, 200, 0)
            else:
                status_text = f"SEARCHING TARGET [ID: {self.selected_target_id}]..."
                badge_color = (0, 140, 255)
        else:
            status_text = "STANDBY: CLICK PERSON OR PRESS [1-9] TO SELECT TARGET"
            badge_color = (0, 220, 255)

        (sw, sh), _ = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (10, 10), (20 + sw, 20 + sh + 6), (30, 30, 30), -1)
        cv2.rectangle(img, (10, 10), (20 + sw, 20 + sh + 6), badge_color, 2)
        cv2.putText(img, status_text, (15, 16 + sh), cv2.FONT_HERSHEY_SIMPLEX, 0.5, badge_color, 1, cv2.LINE_AA)

        # Bottom help instruction bar
        help_text = "Select: Click Box or Press 1/2 | Deselect/Hover: Press 0 or SPACE"
        cv2.rectangle(img, (10, ih - 30), (iw - 10, ih - 8), (20, 20, 20), -1)
        cv2.putText(img, help_text, (16, ih - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)

        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

    def log_metrics(self):
        now = time.time()
        if now - self.last_log < self.log_period:
            return
        self.last_log = now
        rate = 100.0 * self.n_detected / self.n_frames if self.n_frames else 0.0
        self.get_logger().info(
            'state=%s track_id=%s frames=%d det_rate=%.1f%% mean_latency=%.3fs '
            'max_latency=%.3fs switches=%d lost_events=%d'
            % (self.state, self.target_id, self.n_frames, rate,
               self.sum_latency / max(1, self.n_frames), self.max_latency,
               self.n_track_switch, self.n_lost_events))


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
