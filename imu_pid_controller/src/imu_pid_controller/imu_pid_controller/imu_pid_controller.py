import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import math
import time
from rclpy.parameter import Parameter

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

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

        return self.kp * error + self.ki * self.integral + self.kd * derivative


class ImuPidController(Node):
    def __init__(self):
        super().__init__('imu_pid_controller')

        # Parameters
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('yaw_ref', Parameter.Type.DOUBLE)

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.pid = PID(kp, ki, kd)

        self.yaw_ref = None
        self.current_yaw = 0.0
        self.target_linear = 0.0

        # Subscribers & Publishers
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_out', 10)

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('IMU PID controller node started')

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        if self.yaw_ref is None:
            self.yaw_ref = yaw  # set initial heading as reference

    def cmd_callback(self, msg):
        self.target_linear = msg.linear.x

    def update(self):
        if self.yaw_ref is None:
            return

        yaw_error = self.yaw_ref - self.current_yaw
        # Wrap error into [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        correction = self.pid.compute(yaw_error)

        cmd = Twist()
        cmd.linear.x = self.target_linear
        cmd.angular.z = correction
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
