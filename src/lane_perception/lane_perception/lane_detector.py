import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge

import cv2
import numpy as np


class LaneDetector(Node):

    def __init__(self):
        super().__init__("lane_detector")

        # =====================================================
        # Parameters
        # =====================================================

        self.lane_width = 600

        # HLS saturation threshold
        self.saturation_threshold = 80

        # Hough parameters
        self.hough_threshold = 20
        self.min_line_length = 25
        self.max_line_gap = 40

        # Minimum line angle.
        # We deliberately keep this LOW because the right lane
        # can appear relatively flat due to perspective.
        self.min_angle = 5.0

        # =====================================================
        # ROS
        # =====================================================

        self.bridge = CvBridge()

        self.image_subscription = self.create_subscription(
            Image,
            "/prius/front_camera/image_raw",
            self.image_callback,
            10
        )

        self.error_publisher = self.create_publisher(
            Float32,
            "/lane/error",
            10
        )

        self.get_logger().info(
            "Lane detector started!"
        )

    # =========================================================
    # IMAGE CALLBACK
    # =========================================================

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        roi = self.get_roi(frame)

        saturation = self.get_hls_saturation(roi)

        mask = self.create_mask(saturation)

        lines = self.detect_lines(mask)

        left_line, right_line = self.classify_lines(
            lines,
            roi.shape[1]
        )

        left_center = self.get_line_center(
            left_line
        )

        right_center = self.get_line_center(
            right_line
        )

        lane_center, detection_mode = self.calculate_lane_center(
            left_center,
            right_center,
            roi.shape[1]
        )

        error = self.calculate_error(
            lane_center,
            roi.shape[1]
        )

        result = self.draw_detection(
            roi,
            lines,
            left_line,
            right_line,
            lane_center,
            detection_mode,
            error
        )

        self.publish_error(error)

        cv2.imshow(
            "HLS Saturation",
            saturation
        )

        cv2.imshow(
            "Lane Detection",
            result
        )

        cv2.waitKey(1)

    # =========================================================
    # ROI
    # =========================================================

    def get_roi(self, frame):

        height, width = frame.shape[:2]

        # Keep the road area.
        #
        # We deliberately do not crop too aggressively because
        # the lane lines can appear relatively high in the image
        # when the car approaches a curve.
        top = int(height * 0.45)
        bottom = int(height * 0.85)

        return frame[top:bottom, :]

    # =========================================================
    # HLS SATURATION
    # =========================================================

    def get_hls_saturation(self, roi):

        hls = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HLS
        )

        saturation = hls[:, :, 2]

        return saturation

    # =========================================================
    # CREATE MASK
    # =========================================================

    def create_mask(self, saturation):

        _, mask = cv2.threshold(
            saturation,
            self.saturation_threshold,
            255,
            cv2.THRESH_BINARY
        )

        # Remove tiny isolated noise.
        kernel = np.ones(
            (3, 3),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        # Connect small gaps in the lane markings.
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        return mask

    # =========================================================
    # HOUGH LINE DETECTION
    # =========================================================

    def detect_lines(self, mask):

        lines = cv2.HoughLinesP(
            mask,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )

        return lines

    # =========================================================
    # LINE CLASSIFICATION
    # =========================================================

    def classify_lines(
        self,
        lines,
        roi_width
    ):

        left_candidates = []
        right_candidates = []

        if lines is None:
            return None, None

        image_center = roi_width / 2

        for line_data in lines:

            x1, y1, x2, y2 = line_data[0]

            dx = x2 - x1
            dy = y2 - y1

            # Avoid vertical division problems.
            if abs(dx) < 1:
                continue

            length = np.sqrt(
                dx ** 2 + dy ** 2
            )

            if length < self.min_line_length:
                continue

            slope = dy / dx

            angle = np.degrees(
                np.arctan2(
                    dy,
                    dx
                )
            )

            # Normalize angle to -90 ... 90
            if angle > 90:
                angle -= 180

            if angle < -90:
                angle += 180

            if abs(angle) < self.min_angle:
                continue

            # -------------------------------------------------
            # Midpoint of line
            # -------------------------------------------------

            midpoint_x = (
                x1 + x2
            ) / 2

            midpoint_y = (
                y1 + y2
            ) / 2

            # -------------------------------------------------
            # LEFT LINE
            #
            # In image coordinates the left lane generally
            # has a negative slope.
            #
            # We ALSO require its midpoint to be left of the
            # image center.
            # -------------------------------------------------

            if slope < -0.15:

                if midpoint_x < image_center:

                    left_candidates.append(
                        (
                            line_data[0],
                            length,
                            midpoint_x,
                            midpoint_y
                        )
                    )

            # -------------------------------------------------
            # RIGHT LINE
            #
            # The right lane generally has a positive slope.
            #
            # IMPORTANT:
            # We intentionally use +0.05 rather than +0.25.
            #
            # The Prius camera perspective can make the right
            # lane appear almost horizontal.
            # -------------------------------------------------

            elif slope > 0.05:

                if midpoint_x > image_center:

                    right_candidates.append(
                        (
                            line_data[0],
                            length,
                            midpoint_x,
                            midpoint_y
                        )
                    )

        left_line = self.select_best_line(
            left_candidates,
            prefer_left=True,
            image_center=image_center
        )

        right_line = self.select_best_line(
            right_candidates,
            prefer_left=False,
            image_center=image_center
        )

        return left_line, right_line

    # =========================================================
    # SELECT BEST LINE
    # =========================================================

    def select_best_line(
        self,
        candidates,
        prefer_left,
        image_center
    ):

        if not candidates:
            return None

        # Score candidates.
        #
        # We want:
        # - long lines
        # - lines clearly on their respective side
        #
        # This prevents random small yellow objects from winning.

        best_line = None
        best_score = -float("inf")

        for line, length, midpoint_x, midpoint_y in candidates:

            distance_from_center = abs(
                midpoint_x - image_center
            )

            length_score = length

            position_score = distance_from_center

            score = (
                length_score
                + position_score * 0.5
            )

            if score > best_score:

                best_score = score
                best_line = line

        return best_line

    # =========================================================
    # LINE CENTER
    # =========================================================

    def get_line_center(self, line):

        if line is None:
            return None

        x1, y1, x2, y2 = line

        center_x = int(
            (x1 + x2) / 2
        )

        center_y = int(
            (y1 + y2) / 2
        )

        return (
            center_x,
            center_y
        )

    # =========================================================
    # LANE CENTER
    # =========================================================

    def calculate_lane_center(
        self,
        left_center,
        right_center,
        roi_width
    ):

        image_center = roi_width // 2

        # -----------------------------------------------------
        # BOTH LANES
        # -----------------------------------------------------

        if (
            left_center is not None
            and right_center is not None
        ):

            lane_center_x = int(
                (
                    left_center[0]
                    + right_center[0]
                ) / 2
            )

            lane_center_y = int(
                (
                    left_center[1]
                    + right_center[1]
                ) / 2
            )

            return (
                lane_center_x,
                lane_center_y
            ), "Both lanes"

        # -----------------------------------------------------
        # LEFT ONLY
        # -----------------------------------------------------

        if left_center is not None:

            estimated_right_x = (
                left_center[0]
                + self.lane_width
            )

            lane_center_x = int(
                (
                    left_center[0]
                    + estimated_right_x
                ) / 2
            )

            lane_center_y = left_center[1]

            return (
                lane_center_x,
                lane_center_y
            ), "Left lane only"

        # -----------------------------------------------------
        # RIGHT ONLY
        # -----------------------------------------------------

        if right_center is not None:

            estimated_left_x = (
                right_center[0]
                - self.lane_width
            )

            lane_center_x = int(
                (
                    estimated_left_x
                    + right_center[0]
                ) / 2
            )

            lane_center_y = right_center[1]

            return (
                lane_center_x,
                lane_center_y
            ), "Right lane only"

        # -----------------------------------------------------
        # NOTHING
        # -----------------------------------------------------

        return None, "No lane"

    # =========================================================
    # ERROR
    # =========================================================

    def calculate_error(
        self,
        lane_center,
        roi_width
    ):

        if lane_center is None:
            return None

        image_center = roi_width // 2

        return float(
            lane_center[0]
            - image_center
        )

    # =========================================================
    # PUBLISH ERROR
    # =========================================================

    def publish_error(self, error):

        if error is None:
            return

        msg = Float32()

        msg.data = error

        self.error_publisher.publish(
            msg
        )

    # =========================================================
    # DRAW DETECTION
    # =========================================================

    def draw_detection(
        self,
        roi,
        lines,
        left_line,
        right_line,
        lane_center,
        detection_mode,
        error
    ):

        result = roi.copy()

        roi_height, roi_width = result.shape[:2]

        image_center = roi_width // 2

        # -----------------------------------------------------
        # Draw ALL Hough candidates faintly
        # -----------------------------------------------------

        if lines is not None:

            for line_data in lines:

                x1, y1, x2, y2 = line_data[0]

                cv2.line(
                    result,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (100, 100, 100),
                    1
                )

        # -----------------------------------------------------
        # Draw selected LEFT line
        # -----------------------------------------------------

        if left_line is not None:

            x1, y1, x2, y2 = left_line

            cv2.line(
                result,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (255, 0, 255),
                4
            )

        # -----------------------------------------------------
        # Draw selected RIGHT line
        # -----------------------------------------------------

        if right_line is not None:

            x1, y1, x2, y2 = right_line

            cv2.line(
                result,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (255, 0, 255),
                4
            )

        # -----------------------------------------------------
        # Draw estimated missing lane
        # -----------------------------------------------------

        if detection_mode == "Left lane only":

            left_x = left_line[0]

            left_y = left_line[1]

            right_x = (
                int(left_x)
                + self.lane_width
            )

            cv2.line(
                result,
                (
                    right_x,
                    int(left_y)
                ),
                (
                    right_x,
                    roi_height
                ),
                (0, 165, 255),
                2
            )

        elif detection_mode == "Right lane only":

            right_x = right_line[0]

            right_y = right_line[1]

            left_x = (
                int(right_x)
                - self.lane_width
            )

            cv2.line(
                result,
                (
                    left_x,
                    int(right_y)
                ),
                (
                    left_x,
                    roi_height
                ),
                (0, 165, 255),
                2
            )

        # -----------------------------------------------------
        # Image center
        # -----------------------------------------------------

        cv2.line(
            result,
            (
                image_center,
                0
            ),
            (
                image_center,
                roi_height
            ),
            (255, 255, 255),
            2
        )

        # -----------------------------------------------------
        # Lane center
        # -----------------------------------------------------

        if lane_center is not None:

            cv2.circle(
                result,
                lane_center,
                10,
                (0, 255, 255),
                -1
            )

            cv2.line(
                result,
                lane_center,
                (
                    image_center,
                    lane_center[1]
                ),
                (0, 255, 255),
                2
            )

            cv2.putText(
                result,
                f"Error: {error:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2
            )

        else:

            cv2.putText(
                result,
                "No lane detected",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

        # -----------------------------------------------------
        # Detection mode
        # -----------------------------------------------------

        cv2.putText(
            result,
            detection_mode,
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        return result


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = LaneDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        cv2.destroyAllWindows()

        rclpy.shutdown()


if __name__ == "__main__":
    main()