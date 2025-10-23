import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time

class PID:
    def __init__(self, kp, ki, kd, max_output=None, min_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        self.max_output = max_output
        self.min_output = min_output

    def compute(self, error):
        now = time.time()
        if self.prev_time is None:
            self.prev_time = now
            return 0.0

        dt = now - self.prev_time
        self.prev_time = now

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.max_output is not None:
            output = min(self.max_output, output)
        if self.min_output is not None:
            output = max(self.min_output, output)

        return output


class AngularPIDController(Node):
    def __init__(self):
        super().__init__('angular_pid_controller')

        # PID parameters
        self.pid = PID(kp=1.4, ki=0.1, kd=0.001, max_output=2.0, min_output=-2.0)

        self.target_linear = 0.0   # linear.x from /cmd_vel
        self.target_angular = 0.0  # angular.z from /cmd_vel
        self.current_angular = 0.0 # yaw rate from IMU

        # Subscribers & Publisher
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_out', 10)

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('Angular PID controller node started')

    def cmd_callback(self, msg):
        # Pass through linear velocity
        self.target_linear = msg.linear.x
        # Target angular velocity to track with PID
        self.target_angular = msg.angular.z

    def imu_callback(self, msg):
        # IMU angular velocity around Z axis (yaw)
        self.current_angular = msg.angular_velocity.z

    def update(self):
        # Compute PID correction for angular velocity
        angular_error = self.target_angular - self.current_angular
        correction = self.pid.compute(angular_error)

        # Publish corrected cmd_vel
        cmd = Twist()
        cmd.linear.x = self.target_linear      # original linear velocity
        cmd.angular.z = correction             # PID-corrected angular
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AngularPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
