import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LaneController(Node):

    def __init__(self):
        super().__init__("lane_controller")

        # -------------------------
        # Controller parameters
        # -------------------------

        # PD gains
        self.kp = 0.02
        self.kd = 0.0005

        self.forward_speed = 5.0
        self.steering_limit = 0.7

        # -------------------------
        # Controller state
        # -------------------------

        self.previous_error = 0.0
        self.last_time = self.get_clock().now()

        # -------------------------
        # ROS communication
        # -------------------------

        self.error_subscription = self.create_subscription(
            Float32,
            "/lane/error",
            self.error_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.get_logger().info(
            "Lane controller started!"
        )

    # =========================================================
    # Main callback
    # =========================================================

    def error_callback(self, msg):

        error = msg.data

        dt = self.calculate_dt()

        derivative = self.calculate_derivative(
            error,
            dt
        )

        steering = self.calculate_steering(
            error,
            derivative
        )

        self.publish_velocity(steering)

        self.update_controller_state(error)

        self.log_controller_state(
            error,
            derivative,
            steering
        )

    # =========================================================
    # Time calculation
    # =========================================================

    def calculate_dt(self):

        current_time = self.get_clock().now()

        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9

        if dt <= 0.0:
            dt = 0.001

        return dt

    # =========================================================
    # Derivative calculation
    # =========================================================

    def calculate_derivative(self, error, dt):

        derivative = (
            error - self.previous_error
        ) / dt

        return derivative

    # =========================================================
    # Steering calculation
    # =========================================================

    def calculate_steering(
        self,
        error,
        derivative
    ):

        steering = -(
            self.kp * error
            + self.kd * derivative
        )

        # Limit steering
        steering = max(
            -self.steering_limit,
            min(
                self.steering_limit,
                steering
            )
        )

        return steering

    # =========================================================
    # Vehicle control
    # =========================================================

    def publish_velocity(self, steering):

        cmd = Twist()

        cmd.linear.x = self.forward_speed
        cmd.angular.z = steering

        self.cmd_publisher.publish(cmd)

    # =========================================================
    # State management
    # =========================================================

    def update_controller_state(self, error):

        self.previous_error = error
        self.last_time = self.get_clock().now()

    # =========================================================
    # Logging
    # =========================================================

    def log_controller_state(
        self,
        error,
        derivative,
        steering
    ):

        self.get_logger().info(
            f"Error: {error:.2f} | "
            f"Derivative: {derivative:.2f} | "
            f"Steering: {steering:.3f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = LaneController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()