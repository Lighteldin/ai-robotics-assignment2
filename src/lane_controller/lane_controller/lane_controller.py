import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LaneController(Node):

    def __init__(self):
        super().__init__("lane_controller")

        self.error = 0.0
        self.previous_error = 0.0
        self.integral_error = 0.0

        self.last_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Float32,
            "/lane/error",
            self.error_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # PID gains
        self.kp = 0.01
        self.ki = 0.00002
        self.kd = 0.0005

        # Prevent integral windup
        self.integral_limit = 1000.0

        # Limit steering
        self.steering_limit = 0.5

        self.get_logger().info("Lane controller started!")

    def error_callback(self, msg):

        self.error = msg.data

        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Protect against invalid/small dt
        if dt <= 0.0:
            dt = 0.001

        # Integral term
        self.integral_error += self.error * dt

        # Prevent integral windup
        self.integral_error = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral_error)
        )

        # Derivative term
        derivative_error = (
            self.error - self.previous_error
        ) / dt

        # PID controller
        steering = -(
            self.kp * self.error
            + self.ki * self.integral_error
            + self.kd * derivative_error
        )

        # Limit steering
        steering = max(
            -self.steering_limit,
            min(self.steering_limit, steering)
        )

        # Update previous values
        self.previous_error = self.error
        self.last_time = current_time

        # Create velocity command
        cmd = Twist()

        # Forward speed
        cmd.linear.x = 2.0

        # Steering
        cmd.angular.z = steering

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Error: {self.error:.2f} | "
            f"Integral: {self.integral_error:.2f} | "
            f"Derivative: {derivative_error:.2f} | "
            f"Steering: {steering:.3f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = LaneController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()