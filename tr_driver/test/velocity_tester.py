
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import sys
import termios
import tty

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        self.publisher_ = self.create_publisher(JointState, 'joint_desired', 10)
        self.get_logger().info("Motor Tester Node Started. Use the keys to send commands.")

    def send_velocity_command(self, motor1_vel, motor2_vel):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['driver1_motor1', 'driver1_motor2']
        # Ensure velocity is a list of floats
        msg.velocity = [float(motor1_vel), float(motor2_vel)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: name={msg.name}, velocity={msg.velocity}')

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_instructions():
    print("""
    -------------------------------------------
    Reading from the keyboard
    -------------------------------------------
    w/s: increase/decrease linear velocity
    a/d: increase/decrease angular velocity
    space key, x: force stop
    
    q to quit
    -------------------------------------------
    """)

def main(args=None):
    # This script requires the 'select' module, which is standard on Linux/macOS
    # but not on Windows.
    if sys.platform == "win32":
        print("This script is not compatible with Windows.")
        return

    global select
    import select

    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    motor_tester = MotorTester()

    speed = 0.0
    turn = 0.0
    speed_limit = 2000.0 # Adjust as needed
    turn_limit = 1000.0  # Adjust as needed
    speed_step = 100.0
    turn_step = 50.0

    try:
        print_instructions()
        while True:
            key = get_key(settings)
            if key == 'w':
                speed = min(speed + speed_step, speed_limit)
            elif key == 's':
                speed = max(speed - speed_step, -speed_limit)
            elif key == 'a':
                turn = min(turn + turn_step, turn_limit)
            elif key == 'd':
                turn = max(turn - turn_step, -turn_limit)
            elif key == ' ' or key == 'x':
                speed = 0.0
                turn = 0.0
            elif key == 'q':
                break
            else:
                # If no key is pressed, continue with the current speed/turn
                pass

            # Convert linear and angular velocities to individual motor velocities
            # This is a simplified differential drive model
            left_vel = speed - turn
            right_vel = speed + turn
            
            motor_tester.send_velocity_command(left_vel, right_vel)
            
            # Add a small delay to control the publishing rate
            time.sleep(0.1)

    except Exception as e:
        print(e)

    finally:
        # Stop the motors before exiting
        motor_tester.send_velocity_command(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        motor_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
