import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import can
import cantools

class CAN_RX_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    def init_can_publisher(self, topic, timer_period):
        self.can_publisher = self.create_publisher(rosarray, topic, 100)
        self.can_timer = self.create_timer(timer_period, self.publish_can_data)

        # Open CAN interface **once** (faster)
        self.bus = can.interface.Bus('can0', bustype='socketcan', bitrate=500000)
        self.db = cantools.database.load_file('/home/agnirath/Agnirath25/src/can_package/DBC_files/can.dbc')

    def publish_can_data(self):
        # Non-blocking CAN message reception
        response = self.bus.recv(timeout=0.001)  # Faster timeout

        if response is None:
            return  # Skip if no message received

        try:
            decoded_data = self.db.decode_message(response.arbitration_id, response.data)
            message_decoded = []
            for i in decoded_data.values():
                message_decoded.append(float(i))
            message_decoded.insert(0, float(response.arbitration_id))
            signal_names = list(decoded_data.keys())

            # Publish only if valid data is received
            can_pub_msg = rosarray()
            can_pub_msg.data = message_decoded
            self.can_publisher.publish(can_pub_msg)

            print("PUB:", message_decoded)
            print(signal_names)
            print("------------------")

        except KeyError:
            pass  # Ignore unknown CAN messages

def main(args=None):
    rclpy.init(args=args)

    can_node = CAN_RX_NODE("can_rx_node")
    can_node.init_can_publisher("can_rx_data", 0.01)

    rclpy.spin(can_node)  # More efficient than `while` loop

    # Cleanup
    can_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
