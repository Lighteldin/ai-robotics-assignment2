import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

from std_msgs.msg import Float32


class LaneDetector(Node):

    def __init__(self):
        super().__init__("lane_detector")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/prius/front_camera/image_raw",
            self.image_callback,
            10
        )

        self.error_pub = self.create_publisher(
            Float32,
            "/lane/error",
            10
        )

        # Approximate distance between the two lane lines
        # in the ROI, measured in pixels.
        self.LANE_WIDTH = 300

        self.get_logger().info("Lane detector started!")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        height, width = frame.shape[:2]

        # Crop road only
        roi = frame[int(height * 0.45):int(height * 0.78), :]

        # HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Saturation channel
        saturation = hsv[:, :, 1]

        # Threshold
        _, mask = cv2.threshold(
            saturation,
            80,
            255,
            cv2.THRESH_BINARY
        )

        # Morphology
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        result = roi.copy()

        left_points = []
        right_points = []

        roi_width = roi.shape[1]

        # --------------------------------------------------
        # Detect lane-line contours
        # --------------------------------------------------

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < 20:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            cv2.rectangle(
                result,
                (x, y),
                (x + w, y + h),
                (255, 0, 0),
                2
            )

            cx = x + w // 2
            cy = y + h // 2

            cv2.circle(
                result,
                (cx, cy),
                5,
                (0, 0, 255),
                -1
            )

            # Separate left and right lane boundaries
            if cx < roi_width // 2:
                left_points.append((cx, cy))
            else:
                right_points.append((cx, cy))

        # --------------------------------------------------
        # Calculate left lane center
        # --------------------------------------------------

        left_center = None

        if left_points:

            left_center = (
                int(np.mean([p[0] for p in left_points])),
                int(np.mean([p[1] for p in left_points]))
            )

            cv2.circle(
                result,
                left_center,
                10,
                (255, 0, 255),
                -1
            )

        # --------------------------------------------------
        # Calculate right lane center
        # --------------------------------------------------

        right_center = None

        if right_points:

            right_center = (
                int(np.mean([p[0] for p in right_points])),
                int(np.mean([p[1] for p in right_points]))
            )

            cv2.circle(
                result,
                right_center,
                10,
                (255, 0, 255),
                -1
            )

        # --------------------------------------------------
        # Image center
        # --------------------------------------------------

        image_center_x = roi_width // 2

        cv2.line(
            result,
            (image_center_x, 0),
            (image_center_x, roi.shape[0]),
            (255, 255, 255),
            2
        )

        # --------------------------------------------------
        # Calculate lane center
        # --------------------------------------------------

        lane_center = None
        detection_mode = "No lane"

        # CASE 1:
        # Both lane lines detected
        if left_center is not None and right_center is not None:

            lane_center_x = int(
                (left_center[0] + right_center[0]) / 2
            )

            lane_center_y = int(
                (left_center[1] + right_center[1]) / 2
            )

            lane_center = (
                lane_center_x,
                lane_center_y
            )

            detection_mode = "Both lanes"

        # CASE 2:
        # Only left lane detected
        elif left_center is not None:

            estimated_right_x = (
                left_center[0] + self.LANE_WIDTH
            )

            lane_center_x = int(
                (left_center[0] + estimated_right_x) / 2
            )

            lane_center_y = left_center[1]

            lane_center = (
                lane_center_x,
                lane_center_y
            )

            detection_mode = "Left lane only"

            # Draw estimated right lane
            cv2.line(
                result,
                (
                    estimated_right_x,
                    0
                ),
                (
                    estimated_right_x,
                    roi.shape[0]
                ),
                (0, 165, 255),
                2
            )

        # CASE 3:
        # Only right lane detected
        elif right_center is not None:

            estimated_left_x = (
                right_center[0] - self.LANE_WIDTH
            )

            lane_center_x = int(
                (estimated_left_x + right_center[0]) / 2
            )

            lane_center_y = right_center[1]

            lane_center = (
                lane_center_x,
                lane_center_y
            )

            detection_mode = "Right lane only"

            # Draw estimated left lane
            cv2.line(
                result,
                (
                    estimated_left_x,
                    0
                ),
                (
                    estimated_left_x,
                    roi.shape[0]
                ),
                (0, 165, 255),
                2
            )

        # --------------------------------------------------
        # Calculate and publish error
        # --------------------------------------------------

        if lane_center is not None:

            cv2.circle(
                result,
                lane_center,
                10,
                (0, 255, 255),
                -1
            )

            # Draw error from image center to lane center
            cv2.line(
                result,
                lane_center,
                (
                    image_center_x,
                    lane_center[1]
                ),
                (0, 255, 255),
                2
            )

            error = float(
                lane_center[0] - image_center_x
            )

            # Publish error
            error_msg = Float32()
            error_msg.data = error

            self.error_pub.publish(error_msg)

            # Display error
            cv2.putText(
                result,
                f"Error: {error:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2
            )

            cv2.putText(
                result,
                detection_mode,
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

        else:

            # No lane detected
            cv2.putText(
                result,
                "No lane detected",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

        # --------------------------------------------------
        # Display
        # --------------------------------------------------

        cv2.imshow("Original", frame)
        cv2.imshow("ROI", roi)
        cv2.imshow("Saturation", saturation)
        cv2.imshow("Mask", mask)
        cv2.imshow("Lane Detection", result)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = LaneDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == "__main__":
    main()