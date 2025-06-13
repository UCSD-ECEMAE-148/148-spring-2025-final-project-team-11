import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ucsd_robocar_actuator2_pkg.vesc_submodule.vesc_client import VESC_

NODE_NAME = 'vesc_node'

class VescNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.subscription = self.create_subscription(
            String,
            'balloon_detections',
            self.listener_callback,
            1
        )

        self.get_logger().info("Vesc Node started")

        #to put lane following to sleep
        self.camera_control_pub = self.create_publisher(String, 'lane_guidance_control', 1)

        #initialize Vesc client
        self.vesc = VESC_()
        # Default actuator values
        self.default_rpm_value = int(10000)
        self.default_steering_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.default_throttle_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
        self.default_max_right_steering = 0.8
        self.servo_right = 0.7
        self.default_straight_steering = 0.5
        self.servo_straight = 0.5
        self.default_max_left_steering = 0.1
        self.servo_left = 0.3
        self.default_zero_throttle = 0.35
        self.default_max_throttle = 0.35
        self.default_min_throttle = 0.35
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_rpm', self.default_rpm_value),
                ('steering_polarity', self.default_steering_polarity),
                ('throttle_polarity', self.default_throttle_polarity),
                ('max_right_steering', self.default_max_right_steering),
                ('straight_steering', self.default_straight_steering),
                ('max_left_steering', self.default_max_left_steering),
                ('zero_throttle', self.default_zero_throttle),
                ('max_throttle', self.default_max_throttle),
                ('min_throttle', self.default_min_throttle)
            ])
        self.max_rpm = int(self.get_parameter('max_rpm').value)
        self.steering_polarity = int(self.get_parameter('steering_polarity').value)
        self.throttle_polarity = int(self.get_parameter('throttle_polarity').value)
        self.max_right_steering = self.get_parameter('max_right_steering').value
        self.straight_steering = self.get_parameter('straight_steering').value
        self.max_left_steering = self.get_parameter('max_left_steering').value
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value

        self.max_right_steering_angle = self.remap(self.max_right_steering)
        self.steering_offset = self.remap(self.straight_steering) - self.default_straight_steering
        self.max_left_steering_angle = self.remap(self.max_left_steering)
        self.zero_rpm = int(self.zero_throttle * self.max_rpm)
        self.max_rpm = int(self.max_throttle  * self.max_rpm)
        self.min_rpm = int(self.min_throttle  * self.max_rpm)
        self.man_rpm = 3500
        self.last_command = ''

    def pause_camera(self):
        msg = String()
        msg.data = 'pause'
        self.camera_control_pub.publish(msg)
        self.get_logger().info('published PAUSE')

        time.sleep(0.1)

    def resume_camera(self):
        msg = String()
        msg.data = 'resume'
        self.camera_control_pub.publish(msg)
        self.get_logger().info('publishes RESUME')

    def remap(self, value):
        input_start = -1
        input_end = 1
        output_start = 0
        output_end = 1
        normalized_output = float(output_start + (value - input_start) * ((output_end - output_start) / (input_end - input_start)))
        return normalized_output

    def clamp(self, data, upper_bound, lower_bound=None):
            if lower_bound==None:
                lower_bound = -upper_bound # making lower bound symmetric about zero
            if data < lower_bound:
                data_c = lower_bound
            elif data > upper_bound:
                data_c = upper_bound
            else:
                data_c = data
            return data_c

    def listener_callback(self, msg):
        data = msg.data.lower()
        self.get_logger().info(f"[VESC] Received detection: {data}")

        if 'dog' in data and self.last_command != 'dog':
                self.last_command = 'dog'
                self.pause_camera()
                self.spin()
                self.resume_camera()

        elif 'turtle' in data and self.last_command != 'turtle':
                self.last_command = 'turtle'
                self.pause_camera()
                self.turn_around()
                self.resume_camera()

        elif 'swan' in data and self.last_command != 'swan':
                self.last_command = 'swan'
                self.pause_camera()
                self.avoid_right()
                self.resume_camera()

        elif 'snake' in data and self.last_command != 'snake':
                self.last_command = 'snake'
                self.pause_camera()
                self.get_faster()
                self.resume_camera()

        elif 'fish' in data and self.last_command != 'fish':
                self.last_command = 'fish'
                self.pause_camera()
                self.avoid_left()
                self.resume_camera()

        #else:
        #       self.vesc.send_rpm(1000)

    def stop(self):
        #self.vesc.send_rpm(self.stop_rpm)
        #self.get_logger().info("Stop")
        self.vesc.send_rpm(0)
        time.sleep(1.0)
        #self.vesc.send_servo_angle(0)

    def spin(self): #dog
        self.get_logger().info("spin")
        #self.vesc.send_rpm(1000)
        #time.sleep(1.2)
        #self.stop()

        #stop first after recognising animals
        self.stop()

        #self.vesc.send_rpm(self.zero_rpm)
        self.vesc.send_servo_angle(0.8)
        #self.vesc.send_rpm(3500)
        self.vesc.send_rpm(self.man_rpm+5000)

        time.sleep(2.0)

        self.vesc.send_servo_angle(0.8)
        self.vesc.send_rpm(self.man_rpm+4000)

        time.sleep(2.0)

        self.vesc.send_rpm(0)
        self.vesc.send_servo_angle(self.servo_straight)
        time.sleep(0.5)

    def avoid_left(self): #fish
        self.get_logger().info("left")
        #self.vesc.send_rpm(1000)
        #time.sleep(1.2)

        self.stop()

        self.vesc.send_servo_angle(0.30)
        #self.vesc.send_rpm(3500)
        self.vesc.send_rpm(self.man_rpm+750)
        time.sleep(2.5)

        #self.vesc.send_servo_angle(self.servo_straight)
        #time.sleep(1.0)

        self.vesc.send_servo_angle(0.75)
        self.vesc.send_rpm(self.man_rpm+1000)
        time.sleep(2.5)

        self.vesc.send_rpm(0)
        self.vesc.send_servo_angle(self.servo_straight)
        time.sleep(0.5)


    def avoid_right(self): #swan
        self.get_logger().info("right")
        #self.vesc.send_rpm(1000)
        #time.sleep(2)

        self.stop()

        self.vesc.send_servo_angle(0.65)
        self.vesc.send_rpm(self.man_rpm+500)
        time.sleep(2.5)

        #self.vesc.send_servo_angle(self.servo_straight)
        #time.sleep(1.0)

        self.vesc.send_servo_angle(0.25)
        self.vesc.send_rpm(self.man_rpm+2000)
        time.sleep(2.5)

        self.vesc.send_rpm(0)
        self.vesc.send_servo_angle(self.servo_straight)
        time.sleep(0.5)

        #self.vesc.send_rpm(self.stop_rpm)

    def turn_around(self): #turtle
        self.get_logger().info("turn")
        #self.vesc.send_rpm(5000)

        self.stop()

        self.vesc.send_servo_angle(0.8)
        self.vesc.send_rpm(-self.man_rpm-2600)
        time.sleep(2.0)
        self.vesc.send_servo_angle(0.3)
        self.vesc.send_rpm(self.man_rpm+200)
        time.sleep(2.0)
        self.vesc.send_servo_angle(self.servo_straight)
        self.vesc.send_rpm(0)

    def get_faster(self): #snake
        self.get_logger().info("faster")
        #self.vesc.send_rpm(1000)
        #time.sleep(1.2)

        self.stop()
        
        self.vesc.send_servo_angle(0.5)
        self.vesc.send_rpm(8000)
        time.sleep(2)
        self.vesc.send_rpm(0)

def main(args=None):
    rclpy.init(args=args)
    vesc_node = VescNode()
    try:
        rclpy.spin(vesc_node)
    except KeyboardInterrupt:
        vesc_node.get_logger().info(f'Shutting down {NODE_NAME}...')
    finally:
        vesc_node.destroy_node()
        rclpy.shutdown()
        vesc_node.get_logger().info(f'{NODE_NAME} shut down successfully.')

if __name__ == '__main__':
    main()