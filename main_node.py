import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random

class MAIN_NODE(Node):
    def __init__(self, node):
        super().__init__(node)
        self.final_pub_data = None

    def init_can_subscriber(self, topic):
        self.can_subscriber = self.create_subscription(
            rosarray, topic, self.receive_can_data, 150
        )
        self.can_subscriber  # prevent unused variable warning
        self.can_sub_data = None

    def receive_can_data(self, msg):
        if msg.data:  # Ensure it's not an empty message
            self.can_sub_data = msg.data
            self.get_logger().info(f"Received CAN data: {self.can_sub_data}")

    def init_gps_subscriber(self, topic):
        self.gps_subscriber = self.create_subscription(
            rosarray, topic, self.receive_gps_data, 150
        )
        self.gps_subscriber  # prevent unused variable warning
        self.gps_sub_data = None

    def receive_gps_data(self, msg):
        self.gps_sub_data = msg.data
    
    def init_imu_subscriber(self, topic):
        self.imu_subscriber = self.create_subscription(
            rosarray, topic, self.receive_imu_data, 150
        )
        self.imu_subscriber  # prevent unused variable warning
        self.imu_sub_data = None

    def receive_imu_data(self, msg):
        self.imu_sub_data = msg.data

    def init_final_data_publisher(self, topic, timer_period):
        self.final_data_publisher = self.create_publisher(rosarray, topic, 150)
        self.final_data_timer = self.create_timer(timer_period, self.publish_final_data)
        self.final_pub_data = None

    def publish_final_data(self):
        if self.final_pub_data is not None:
            self.final_data_pub_msg = rosarray()
            self.final_data_pub_msg.data = self.final_pub_data        
            self.final_data_publisher.publish(self.final_data_pub_msg)
            self.final_pub_data = self.final_data_pub_msg.data
            #print("Final_PUB:", self.final_pub_data)

def main(args=None):
    rclpy.init(args=args)

    main_node = MAIN_NODE("main_node")
    main_node.init_can_subscriber("can_rx_data")
    main_node.init_gps_subscriber("gps_data")
    main_node.init_imu_subscriber("imu_data")
    main_node.init_final_data_publisher("final_data", 1)

    final_data = [0.0]*80

    print("Starting Main Node")
    while rclpy.ok():

        rclpy.spin_once(main_node, timeout_sec = 0.1)

        gps_sub_data = main_node.gps_sub_data
        imu_sub_data = main_node.imu_sub_data
        can_sub_data = main_node.can_sub_data

        bms_fd_index = 0
        mc_fd_index = 24
        mppt1_fd_index = 35
        mppt2_fd_index = 42
        mppt3_fd_index = 49
        mppt4_fd_index = 56
        gps_fd_index = 63
        imu_fd_index = 67

        if can_sub_data is not None:

            if can_sub_data[0] == 1537:
                final_data[bms_fd_index:bms_fd_index+2] = can_sub_data[2:4]
            if can_sub_data[0] == 1540:
                final_data[bms_fd_index+2:bms_fd_index+4] = can_sub_data[2:4]
            if can_sub_data[0] == 1543:
                final_data[bms_fd_index+4:bms_fd_index+6] = can_sub_data[2:4]
            if can_sub_data[0] == 1546:
                final_data[bms_fd_index+6:bms_fd_index+8] = can_sub_data[2:4]
            if can_sub_data[0] == 1549:
                final_data[bms_fd_index+8:bms_fd_index+10] = can_sub_data[2:4]
            if can_sub_data[0] == 1780:
                final_data[bms_fd_index+10] = can_sub_data[1]
            if can_sub_data[0] == 1783:
                final_data[bms_fd_index+11:bms_fd_index+13] = can_sub_data[1:3]
            if can_sub_data[0] == 1786:
                final_data[bms_fd_index+13:bms_fd_index+15] = can_sub_data[1:3]
            if can_sub_data[0] == 1787:
                final_data[bms_fd_index+15:bms_fd_index+23] = can_sub_data[1:9]
            if can_sub_data[0] == 1789:
                final_data[bms_fd_index+23] = can_sub_data[1]

            if can_sub_data[0] == 1025:
                final_data[mc_fd_index+0:mc_fd_index+2] = can_sub_data[1:3]
            if can_sub_data[0] == 1026:
                final_data[mc_fd_index+2:mc_fd_index+4] = can_sub_data[1:3]
            if can_sub_data[0] == 1027:
                final_data[mc_fd_index+4:mc_fd_index+6] = can_sub_data[1:3]
            if can_sub_data[0] == 1028:
                final_data[mc_fd_index+6:mc_fd_index+8] = can_sub_data[1:3]
            if can_sub_data[0] == 1035:
                final_data[mc_fd_index+8:mc_fd_index+10] = can_sub_data[1:3]
            if can_sub_data[0] == 1036:
                final_data[mc_fd_index+10] = can_sub_data[1]
            if can_sub_data[0] == 1038:
                final_data[mc_fd_index+11] = can_sub_data[1]

            if can_sub_data[0] == 1696:
                final_data[mppt1_fd_index+0:mppt1_fd_index+2] = can_sub_data[1:3]
            if can_sub_data[0] == 1697:
                final_data[mppt1_fd_index+2:mppt1_fd_index+4] = can_sub_data[1:3]
            if can_sub_data[0] == 1698:
                final_data[mppt1_fd_index+4:mppt1_fd_index+6] = can_sub_data[1:3]
            if can_sub_data[0] == 1701:
                final_data[mppt1_fd_index+6] = can_sub_data[1]

            if can_sub_data[0] == 1712:
                final_data[mppt2_fd_index+0:mppt2_fd_index+2] = can_sub_data[1:3]
            if can_sub_data[0] == 1713:
                final_data[mppt2_fd_index+2:mppt2_fd_index+4] = can_sub_data[1:3]
            if can_sub_data[0] == 1714:
                final_data[mppt2_fd_index+4:mppt2_fd_index+6] = can_sub_data[1:3]
            if can_sub_data[0] == 1717:
                final_data[mppt2_fd_index+6] = can_sub_data[1]

            if can_sub_data[0] == 1728:
                final_data[mppt3_fd_index+0:mppt3_fd_index+2] = can_sub_data[1:3]
            if can_sub_data[0] == 1729:
                final_data[mppt3_fd_index+2:mppt3_fd_index+4] = can_sub_data[1:3]
            if can_sub_data[0] == 1730:
                final_data[mppt3_fd_index+4:mppt3_fd_index+6] = can_sub_data[1:3]
            if can_sub_data[0] == 1733:
                final_data[mppt3_fd_index+6] = can_sub_data[1]

            if can_sub_data[0] == 1744:
                final_data[mppt4_fd_index+0:mppt4_fd_index+2] = can_sub_data[1:3]
            if can_sub_data[0] == 1745:
                final_data[mppt4_fd_index+2:mppt4_fd_index+4] = can_sub_data[1:3]
            if can_sub_data[0] == 1746:
                final_data[mppt4_fd_index+4:mppt4_fd_index+6] = can_sub_data[1:3]
            if can_sub_data[0] == 1749:
                final_data[mppt4_fd_index+6] = can_sub_data[1]
            
        if gps_sub_data is not None:
            final_data[gps_fd_index:gps_fd_index+3] = gps_sub_data[1:4]
            final_data[gps_fd_index+3] = gps_sub_data[6]

        if imu_sub_data is not None:
            final_data[imu_fd_index:imu_fd_index+3] = imu_sub_data

        if final_data[bms_fd_index+15] == 1.0:
            final_data[bms_fd_index+15] = 8.0

        if final_data[bms_fd_index+16] == 1.0:
            final_data[bms_fd_index+16] = 7.0

        if final_data[bms_fd_index+17] == 1.0:
            final_data[bms_fd_index+17] = 6.0

        if final_data[bms_fd_index+18] == 1.0:
            final_data[bms_fd_index+18] = 5.0

        if final_data[bms_fd_index+19] == 1.0:
            final_data[bms_fd_index+19] = 4.0

        if final_data[bms_fd_index+20] == 1.0:
            final_data[bms_fd_index+20] = 3.0

        if final_data[bms_fd_index+21] == 1.0:
            final_data[bms_fd_index+21] = 2.0

        if final_data[bms_fd_index+22] == 1.0:
            final_data[bms_fd_index+22] = 1.0

        main_node.final_pub_data = final_data
        print(main_node.final_pub_data)

    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


            

