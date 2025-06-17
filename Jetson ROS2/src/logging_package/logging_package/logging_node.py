import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
from datetime import datetime
import os
import sys
import time
from supabase import create_client, Client
import math


class LOGGING_NODE(Node):
    def __init__(self, node):
        super().__init__(node)
        self.final_sub_data = None
        self.url = 'https://uycnoowlhgoyqzidqfct.supabase.co'
        self.key = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InV5Y25vb3dsaGdveXF6aWRxZmN0Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDIyOTYyMDYsImV4cCI6MjA1Nzg3MjIwNn0.n_zkXBA_0ziR869eROLUPqnSE050RvvdtV7tPAlIlZY'
        self.supabase: Client = create_client(self.url,self.key)
        self.timer = self.create_timer(3.0, self.log_data_periodically)

    def init_final_data_subscriber(self, topic):
        self.final_subscriber = self.create_subscription(
            rosarray, topic, self.receive_final_data, 150
        )
        self.get_logger().info(f"Subscribed to topic: {topic}")

    def receive_final_data(self, msg):
        self.final_sub_data = msg.data

    def log_data_periodically(self):

        try:

            if self.final_sub_data is not None and len(self.final_sub_data)==79:

                self.get_logger().info(f"Data Recieved: {self.final_sub_data}")
                table_data = {
                    "Timestamp": datetime.now().isoformat(),
                    "CMU1_Temp": self.final_sub_data[0],
                    "CMU1_Cell_Temp": self.final_sub_data[1],
                    "CMU1_Cell0 (mV)": self.final_sub_data[2],
                    "CMU1_Cell1 (mV)": self.final_sub_data[3],
                    "CMU1_Cell2 (mV)": self.final_sub_data[4],
                    "CMU1_Cell3 (mV)": self.final_sub_data[5],
                    "CMU1_Cell4 (mV)": self.final_sub_data[6],
                    "CMU1_Cell5 (mV)": self.final_sub_data[7],
                    "CMU1_Cell6 (mV)": self.final_sub_data[8],
                    "CMU1_Cell7 (mV)": self.final_sub_data[9],
                    "CMU1_Total_Voltage": sum(self.final_sub_data[2:10])/1000,
                    "CMU2_Temp": self.final_sub_data[10],
                    "CMU2_Cell_Temp": self.final_sub_data[11],
                    "CMU2_Cell0 (mV)": self.final_sub_data[12],
                    "CMU2_Cell1 (mV)": self.final_sub_data[13],
                    "CMU2_Cell2 (mV)": self.final_sub_data[14],
                    "CMU2_Cell3 (mV)": self.final_sub_data[15],
                    "CMU2_Cell4 (mV)": self.final_sub_data[16],
                    "CMU2_Cell5 (mV)": self.final_sub_data[17],
                    "CMU2_Cell6 (mV)": self.final_sub_data[18],
                    "CMU2_Cell7 (mV)": self.final_sub_data[19],
                    "CMU2_Total_Voltage": sum(self.final_sub_data[12:20])/1000,
                    "CMU3_Temp": self.final_sub_data[20],
                    "CMU3_Cell_Temp": self.final_sub_data[21],
                    "CMU3_Cell0 (mV)": self.final_sub_data[22],
                    "CMU3_Cell1 (mV)": self.final_sub_data[23],
                    "CMU3_Cell2 (mV)": self.final_sub_data[24],
                    "CMU3_Cell3 (mV)": self.final_sub_data[25],
                    "CMU3_Cell4 (mV)": self.final_sub_data[26],
                    "CMU3_Cell5 (mV)": self.final_sub_data[27],
                    "CMU3_Cell6 (mV)": self.final_sub_data[28],
                    "CMU3_Cell7 (mV)": self.final_sub_data[29],
                    "CMU3_Total_Voltage": sum(self.final_sub_data[22:30])/1000,
                    "Pack_Voltage (V)": (sum(self.final_sub_data[2:10])+sum(self.final_sub_data[12:20])+sum(self.final_sub_data[22:30]))/1000,
                    "Input_Voltage_A (V)": self.final_sub_data[30],
                    "Input_Current_A (A)": self.final_sub_data[31],
                    "Input_Power_A (W)": self.final_sub_data[30]*self.final_sub_data[31],
                    "Output_Voltage_A (V)": self.final_sub_data[32],
                    "Output_Current_A (A)": self.final_sub_data[33],
                    "Output_Power_A (W)": self.final_sub_data[32]*self.final_sub_data[33],
                    "Mosfet_Temp_A": self.final_sub_data[34],
                    "Controller_Temp_A": self.final_sub_data[35],
                    "Input_Voltage_B (V)": self.final_sub_data[36],
                    "Input_Current_B (A)": self.final_sub_data[37],
                    "Input_Power_B (W)": self.final_sub_data[36]*self.final_sub_data[37],
                    "Output_Voltage_B (V)": self.final_sub_data[38],
                    "Output_Current_B (A)": self.final_sub_data[39],
                    "Output_Power_B (W)": self.final_sub_data[38]*self.final_sub_data[39],
                    "Mosfet_Temp_B": self.final_sub_data[40],
                    "Controller_Temp_B": self.final_sub_data[41],
                    "Input_Voltage_C (V)": self.final_sub_data[42],
                    "Input_Current_C (A)": self.final_sub_data[43],
                    "Input_Power_C (W)": self.final_sub_data[42]*self.final_sub_data[43],
                    "Output_Voltage_C (V)": self.final_sub_data[44],
                    "Output_Current_C (A)": self.final_sub_data[45],
                    "Output_Power_C (W)": self.final_sub_data[44]*self.final_sub_data[45],
                    "Mosfet_Temp_C": self.final_sub_data[46],
                    "Controller_Temp_C": self.final_sub_data[47],
                    "Input_Voltage_D (V)": self.final_sub_data[48],
                    "Input_Current_D (A)": self.final_sub_data[49],
                    "Input_Power_D (W)": self.final_sub_data[48]*self.final_sub_data[49],
                    "Output_Voltage_D (V)": self.final_sub_data[50],
                    "Output_Current_D (A)": self.final_sub_data[51],
                    "Output_Power_D (W)": self.final_sub_data[50]*self.final_sub_data[51],
                    "Mosfet_Temp_D": self.final_sub_data[52],
                    "Controller_Temp_D": self.final_sub_data[53],
                    "Latitude": self.final_sub_data[55],
                    "Longitude": self.final_sub_data[56],
                    "Satellites": self.final_sub_data[59],
                    "Speed (kmph)": self.final_sub_data[66]/20*2*math.pi/60*0.287*3.6,
                    "RPM": self.final_sub_data[66]/20,
                    "MC_Current": self.final_sub_data[67],
                    "Duty_Cycle": self.final_sub_data[68],
                    "Amp_hrs": self.final_sub_data[69],
                    "Amp_hrs_chg": self.final_sub_data[70],
                    "Watt_hrs": self.final_sub_data[71],
                    "Watt_hrs_chg": self.final_sub_data[72],
                    "Temp_Mosfet": self.final_sub_data[73],
                    "Temp_Motor": self.final_sub_data[74],
                    "Current_in": self.final_sub_data[75],
                    "PID_posn": self.final_sub_data[76],
                    "Tachometer": self.final_sub_data[77],
                    "MC_Voltage": self.final_sub_data[78]
                    }
                    
                response = self.supabase.table("Telemetry").insert(table_data).execute()
                self.get_logger().info(f"Data logged: {response}")

            else:
                self.get_logger().warn("Data not ready or incorrect length")

        except Exception as e:
            self.get_logger().error(f"Error logging data: {e}")
            restart_script()

def restart_script():
    """Restarts the script using os.execv()"""
    time.sleep(1)  # Wait before restarting
    print("Restarting script...")
    os.execv(sys.executable, ['python'] + sys.argv)  # Restart the script

def main(args=None):
    rclpy.init(args=args)
    logging_node = LOGGING_NODE("logging_node")
    logging_node.init_final_data_subscriber("final_data")
    while rclpy.ok():
        rclpy.spin_once(logging_node)
    logging_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
