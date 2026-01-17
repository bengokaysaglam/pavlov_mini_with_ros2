#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class KeyboardTeleop(Node):
    
    def __init__(self):
        super().__init__('keyboard_teleop')
    
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.linear_speed = 0.05
        self.angular_speed = 0.3
        
        self.timer = self.create_timer(0.1, self.publish_cmd)
        
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        self.settings = None
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("KEYBOARD TELEOP STARTED")
        self.print_instructions()
    
    def print_instructions(self):
        print("\n" + "="*50)
        print("QUADRUPED KEYBOARD CONTROL")
        print("="*50)
        print("\nControls:")
        print("  W - Forward")
        print("  S - Backward")
        print("  A - Turn Left")
        print("  D - Turn Right")
        print("  X - Stop")
        print("  Q - Quit")
        print("\nPress keys and watch the terminal for feedback!")
        print("="*50 + "\n")
    
    def get_key(self, timeout=0.1):
        if not sys.stdin.isatty():
            return None
        
        tty.setraw(sys.stdin.fileno())
        
        # Check if input is available
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.current_linear_x
        msg.angular.z = self.current_angular_z
        self.publisher_.publish(msg)
    
    def run(self):
        try:
            self.get_logger().info("Press W/A/S/D to move")
            
            while rclpy.ok():
                key = self.get_key(timeout=0.1)
                
                if key:
                    # Process key
                    if key.lower() == 'w':
                        self.current_linear_x = self.linear_speed
                        self.current_angular_z = 0.0
                        self.get_logger().info("FORWARD")
                        
                    elif key.lower() == 's':
                        self.current_linear_x = -self.linear_speed
                        self.current_angular_z = 0.0
                        self.get_logger().info("BACKWARD")
                        
                    elif key.lower() == 'a':
                        self.current_linear_x =  0.0
                        self.current_angular_z = self.angular_speed
                        self.get_logger().info("TURN LEFT")
                        
                    elif key.lower() == 'd':
                        self.current_linear_x = 0.0
                        self.current_angular_z = -self.angular_speed
                        self.get_logger().info("TURN RIGHT")
                        
                    elif key.lower() == 'x':
                        self.current_linear_x = 0.0
                        self.current_angular_z = 0.0
                        self.get_logger().info("STOP")
                        
                    elif key.lower() == 'q':
                        self.get_logger().info("QUITTING...")
                        break
                    
                    elif key == '\x03':  # Ctrl+C
                        break
                
                # Spin once to process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0)
        
        except KeyboardInterrupt:
            pass
        
        finally:
            # Stop robot
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            self.publish_cmd()
            
            if self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            print("\n Teleop stopped\n")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        node.run()
    except Exception as e:
        print(f"\n Error: {e}\n")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()