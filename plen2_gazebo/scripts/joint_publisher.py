import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import sys
import termios
import tty

class TeleopControl(Node):
    def __init__(self):
        super().__init__('teleop_control')
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info("Teleop control node started. Use WASD keys to move and JKL keys to adjust joints.")
        self.run()
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        twist = Twist()
        joint_state = JointState()
        joint_state.name = ['left_shoulder_pitch', 'right_shoulder_pitch']
        joint_state.position = [0.0, 0.0]
        
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':
                twist.linear.x = 0.5
            elif key == 's':
                twist.linear.x = -0.5
            elif key == 'a':
                twist.angular.z = 0.5
            elif key == 'd':
                twist.angular.z = -0.5
            elif key == 'j':
                joint_state.position[0] += 0.1
            elif key == 'k':
                joint_state.position[0] -= 0.1
            elif key == 'l':
                joint_state.position[1] += 0.1
            elif key == 'q':
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            
            self.cmd_publisher.publish(twist)
            self.joint_publisher.publish(joint_state)
        
        self.get_logger().info("Shutting down teleop control.")

if __name__ == '__main__':
    rclpy.init()
    node = TeleopControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

