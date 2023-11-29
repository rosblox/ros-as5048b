import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from winch_control_interfaces.srv import SetFloat32

import yaml
import math

class TurnCounterNode(Node):

    def __init__(self):
        super().__init__('tendon_length_node')
        
        self.declare_parameter('file_path', '/tmp/tendon_length.yaml')
        self.file_path = self.get_parameter('file_path').value

        self.declare_parameter('encoder_topic', '/encoder/deg')
        self.encoder_topic = self.get_parameter('encoder_topic').value

        self.reset_srv = self.create_service(Trigger, '~/reset', self.reset_callback)
        self.calibrate_srv = self.create_service(SetFloat32, '~/calibrate', self.calibrate_callback)

        self.subscription = self.create_subscription(Float32, self.encoder_topic, self.encoder_callback, 20)

        self.publisher = self.create_publisher(Float32, '~/tendon_length', 5)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer_counter = 0

        self.tendon_length = self.load_tendon_length_from_file()
        self.prev_encoder_value = None  
        
        self.publish_tendon_length()


    def calibrate_callback(self, request, response):
        self.tendon_length = request.data
        response.success=True
        return response


    def reset_callback(self, request, response):
        self.get_logger().info(f"Reset tendon length from {self.tendon_length}")
        self.tendon_length = 0.0
        response.success=True
        return response
    

    def timer_callback(self):
        self.publish_tendon_length()
        
        self.timer_counter += 1

        if self.timer_counter == 10:
            self.write_tendon_length_to_file()
            self.timer_counter = 0


    def publish_tendon_length(self):
        msg_out = Float32()
        msg_out.data = self.tendon_length
        self.publisher.publish(msg_out)


    def encoder_callback(self, encoder_msg):

        encoder_value = encoder_msg.data

        if self.prev_encoder_value is not None:
            encoder_movement = self.calculate_movement(encoder_value, self.prev_encoder_value)
            
            ### TODO: Add more sophisticated tendon length model here
            
            drum_diameter=0.01
            drum_circumference=math.pi*drum_diameter
            
            delta_tendon = drum_circumference * encoder_movement/360.0

            self.tendon_length -= delta_tendon

            ### 

        self.prev_encoder_value = encoder_value


    def calculate_movement(self, current_value, previous_value):
        movement = current_value - previous_value

        # Handle the case of a jump from 360 to 0 (or vice versa)
        if movement > 180:
            movement -= 360
        elif movement < -180:
            movement += 360

        return movement


    def load_tendon_length_from_file(self):
        try:
            with open(self.file_path, 'r') as file:
                data = yaml.safe_load(file)
                if 'tendon_length' in data:
                    return data['tendon_length']
        except (FileNotFoundError, yaml.YAMLError, TypeError):
            pass
        return 0.0  # Default value if file doesn't exist or is invalid


    def write_tendon_length_to_file(self):
        data = {'tendon_length': self.tendon_length}
        with open(self.file_path, 'w') as file:
            yaml.dump(data, file)



def main(args=None):
    rclpy.init(args=args)

    turn_counter_node = TurnCounterNode()

    try:
        rclpy.spin(turn_counter_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turn_counter_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
