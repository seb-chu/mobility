import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pigpio  # Raspberry Pi PWM library

# Define GPIO pins for motors (PWM)
PWM_LEFT = 12    # Left motors
PWM_RIGHT = 18   # Right motors

# Define GPIO pins for servos
SERVO_FL = 5  # Front Left Servo
SERVO_FR = 6  # Front Right Servo
SERVO_RL = 13 # Rear Left Servo
SERVO_RR = 19 # Rear Right Servo

# Define PWM values for servo angles (Assuming servo expects 1000-2000µs)
SERVO_MIN = 1000
SERVO_MAX = 2000
SERVO_NEUTRAL = 1500  # Middle position (0°)

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.log = self.get_logger()

        # Initialize PWM via pigpio
        self.pi = pigpio.pi()
        self.pi.set_PWM_frequency(PWM_LEFT, 50)  # Motor PWM at 50Hz
        self.pi.set_PWM_frequency(PWM_RIGHT, 50)
        
        # Set servo PWM frequency
        self.pi.set_PWM_frequency(SERVO_FL, 50)
        self.pi.set_PWM_frequency(SERVO_FR, 50)
        self.pi.set_PWM_frequency(SERVO_RL, 50)
        self.pi.set_PWM_frequency(SERVO_RR, 50)

        # Subscribe to wheel speed commands
        self.create_subscription(Float64MultiArray, "/rover/targets", self.targets_cb, 10)
        
        # Subscribe to servo commands
        self.create_subscription(Float64MultiArray, "/rover/servo_targets", self.servo_cb, 10)

    def targets_cb(self, msg):
        """ Convert velocity commands into PWM signals """
        if len(msg.data) < 4:
            self.log.error("Invalid motor command received!")
            return

        left_speed = msg.data[0]
        right_speed = msg.data[2]

        # Convert speed to PWM (1ms-2ms range)
        left_pwm = self.map_speed_to_pwm(left_speed)
        right_pwm = self.map_speed_to_pwm(right_speed)

        # Send PWM signals
        self.pi.set_servo_pulsewidth(PWM_LEFT, left_pwm)
        self.pi.set_servo_pulsewidth(PWM_RIGHT, right_pwm)

        self.log.info(f"Left PWM: {left_pwm}, Right PWM: {right_pwm}")

    def servo_cb(self, msg):
        """ Convert servo angle commands into PWM """
        if len(msg.data) < 4:
            self.log.error("Invalid servo command received!")
            return

        # Convert degrees to servo pulse width
        servo_FL = self.map_angle_to_pwm(msg.data[0])
        servo_FR = self.map_angle_to_pwm(msg.data[1])
        servo_RL = self.map_angle_to_pwm(msg.data[2])
        servo_RR = self.map_angle_to_pwm(msg.data[3])

        # Send PWM signals to servos
        self.pi.set_servo_pulsewidth(SERVO_FL, servo_FL)
        self.pi.set_servo_pulsewidth(SERVO_FR, servo_FR)
        self.pi.set_servo_pulsewidth(SERVO_RL, servo_RL)
        self.pi.set_servo_pulsewidth(SERVO_RR, servo_RR)

        self.log.info(f"Servo positions: {msg.data}")

    def map_speed_to_pwm(self, speed):
        """ Convert speed (-1 to 1) into PWM range (1000-2000µs) """
        return int((speed + 1) * 500 + 1000)

    def map_angle_to_pwm(self, angle):
        """ Convert angle (-45 to 45 degrees) to PWM (1000-2000µs) """
        return int(SERVO_NEUTRAL + (angle / 45) * 500)

def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
