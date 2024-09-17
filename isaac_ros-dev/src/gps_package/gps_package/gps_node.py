import serial
from sensor_msgs.msg import NavSatFix
import rclpy
from rclpy.node import Node

class GPSNode(Node):
    def __init__(self):
        super().__init__('GPS_publisher')
        
        self.coords_publisher = self.create_publisher(NavSatFix, '/nav/raw/fix', 10)
        
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Open serial port to modem
        try:
            self.gps_serial = serial.Serial('/dev/ttyACM0', 115200)
        except Exception as e:
            self.get_logger().error(f"Serial could not connect: {e}")
            raise e

    def timer_callback(self):
        raw_data = self.gps_serial.readline().decode("utf-8", errors='ignore').strip()
        print(f"Raw message: {raw_data}")

        data = self.extract_fields(raw_data)
        print(f"Extracted data: {data}")

        if len(data) >= 3:
            latitude = data[0]
            longitude = data[1]
            status_code = data[2]

            if status_code != 0 and latitude != 0.0 and longitude != 0.0:
                # GPS fix acquired; publish data
                coords_msg = NavSatFix()
                coords_msg.header.frame_id = 'GPS_coords'
                coords_msg.header.stamp = self.get_clock().now().to_msg()
                coords_msg.latitude = latitude
                coords_msg.longitude = longitude
                self.coords_publisher.publish(coords_msg)
                print(f"Published NavSatFix message with latitude: {latitude}, longitude: {longitude}")
            else:
                print(f"GPS not fixed yet. Status code: {status_code}")
        else:
            print("No valid GPS data to publish.")

    def extract_fields(self, raw_data):
        fields = []
        try:
            data = raw_data.strip().split(',')
            if len(data) >= 3:
                latitude = float(data[0])
                longitude = float(data[1])
                status_code = int(data[2])

                fields.append(latitude)
                fields.append(longitude)
                fields.append(status_code)
                return fields
            else:
                print("BAD MSG: Not enough data fields")
                return fields
        except ValueError as e:
            print(f"BAD MSG: Could not convert data to float - {e}")
            return fields
        except Exception as e:
            print(f"BAD MSG: {e}")
            return fields

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSNode()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
