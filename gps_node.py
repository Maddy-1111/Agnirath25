import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial


class GPS_NODE(Node):
    def __init__(self, node):
        super().__init__(node)
        self.gps_node = node

        # Initialize buffer with default values
        self.buffer = {
            'time_utc': "000000",
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0,
            'hdop': 0.0,
            'satellites': 0.0,
            'speed_knots': 0.0,
            'date': "000000",
            'status': "V",
        }

    def init_gps_publisher(self, topic, timer_period, port, baud_rate):
        self.gps_pub = self.create_publisher(rosarray, topic, 10)
        self.gps_timer = self.create_timer(timer_period, self.parse_gngga)
        self.gps_pub_data = []
        self.port = port
        self.baud_rate = baud_rate
        self.serial_port = None

        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            print("GPS is connected and working")
        except serial.SerialException:
            print("GPS is not working")


    def parse_gngga(self):
        try:
        # Open the serial port
            with serial.Serial(self.port, self.baud_rate) as ser:
                while True:
                    sentence = ser.readline().decode('utf-8').strip()                    
                    if sentence.startswith("$GNGGA"):
                        data = sentence.split(',')
                        # Ensure the sentence has at least 15 fields
                        if len(data) < 15:
                            continue

                        # Satellites
                        self.buffer['satellites'] = float(data[7]) if data[7].isdigit() else self.buffer['satellites']

                        # HDOP
                        self.buffer['hdop'] = float(data[8]) if data[8] else self.buffer['hdop']

                        # Altitude
                        self.buffer['altitude'] = float(data[9]) if data[9] else self.buffer['altitude']


                    if sentence.startswith("$GNRMC"):
                        data = sentence.split(',')
                        # Ensure the sentence has at least 12 fields
                        if len(data) < 12:
                            continue

                        # Extract data from $GNRMC
                        self.buffer['time_utc'] = data[1] if data[1] else self.buffer['time_utc']
                        self.buffer['status'] = data[2] if data[2] else self.buffer['status']  # V = Void (no fix), A = Active (valid fix)
                        
                        if data[3] and data[4]:
                            latitude = float(data[3][:2]) + float(data[3][2:]) / 60.0
                            if data[4] == 'S':
                                latitude = -latitude
                            self.buffer['latitude'] = round(latitude, 6)
                        else:
                            latitude = self.buffer['latitude']

                        if data[5] and data[6]:
                            longitude = float(data[5][:3]) + float(data[5][3:]) / 60.0
                            if data[6] == 'W':
                                longitude = -longitude
                            self.buffer['longitude'] = round(longitude, 6)
                        else:
                            longitude = self.buffer['longitude']


                        self.buffer['speed_knots'] = float(data[7])*1.852 if data[7] else self.buffer['speed_knots'] # Speed in kmph ( 1 knot = 1.852 kmph )
                        self.buffer['date'] = data[9] if data[9] else self.buffer['date']  # Date in DDMMYY format

                    
                    # Print all parsed $GNRMC data
                    print(f"UTC Time: {self.buffer['time_utc']}")
                    print(f"Latitude: {self.buffer['latitude']:.6f}")
                    print(f"Longitude: {self.buffer['longitude']:.6f}")
                    print(f"Altitude: {self.buffer['altitude']} meters")
                    print(f"HDOP: {self.buffer['hdop']}")
                    print(f"Number of Satellites: {self.buffer['satellites']}")
                    print(f"Speed (knots): {self.buffer['speed_knots']}")
                    print(f"Date (DDMMYY): {self.buffer['date']}")
                    print(f"Status: {self.buffer['status']}")
                    print("-----")

                    self.gps_pub_data=[
                        float(self.buffer['time_utc']),
                        self.buffer['latitude'],
                        self.buffer['longitude'],
                        self.buffer['altitude'],
                        self.buffer['hdop'],
                        self.buffer['satellites'],
                        self.buffer['speed_knots'],
                        float(self.buffer['date']),
                        1.0 if self.buffer['status']=='A' else 0.0,
                    ]

                    if self.gps_pub_data is not None:
                        self.gps_pub_msg = rosarray()
                        self.gps_pub_msg.data = self.gps_pub_data
                        self.gps_pub.publish(self.gps_pub_msg)
                        self.gps_pub_data = self.gps_pub_msg.data
                        print("PUB to", self.gps_node, ":", self.gps_pub_data)

            
        except KeyboardInterrupt:
            print("GPS reading stopped.")
        except serial.SerialException as e:
            print(f"Serial port error: {e}")

        

def main(args=None):
    rclpy.init(args=args)

    gps_node = GPS_NODE("gps_node")
    gps_node.init_gps_publisher("gps_data", 1, "/dev/tty_gps", 9600)

    while rclpy.ok():
        rclpy.spin_once(gps_node)

    gps_node.close()
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
