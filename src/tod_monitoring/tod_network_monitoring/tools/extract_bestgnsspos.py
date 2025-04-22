import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import csv

from novatel_oem7_msgs.msg import BESTPOS # ensure this is available in $PYTHONPATH

TOPIC_NAME = '/edgar/sensor/gnss/novatel/center/bestgnsspos'
OUTPUT_FILENAME = 'gnss_output.csv'

"""
Subscribes to a topic upon initialization.
Ensure message type is imported correctly!
After destruction (e.g. Ctrl+C), contents are written to a csv file.
"""
class DataExtractor(Node):
    def __init__(self):
        super().__init__('data_extractor')
        self.subscription = self.create_subscription(
            BESTPOS,
            TOPIC_NAME,
            self.message_callback,
            qos_profile_sensor_data) # ensure QoS settings correctly for sensor data
        self.csv_file = open(OUTPUT_FILENAME, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Latitude', 'Longitude', 'Height'])

    """
    Ensure the fields in the message match the defintion
    """
    def message_callback(self, msg):
        header = msg.header
        timestamp = f"{header.stamp.sec}.{header.stamp.nanosec}"
        latitude = msg.lat
        longitude = msg.lon
        height = msg.hgt

        self.csv_writer.writerow([timestamp, latitude, longitude, height])
        print(f"Saved data to CSV: Timestamp: {timestamp}, Lat: {latitude}, Lon: {longitude}, Hgt: {height}")

    def close(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    extractor = DataExtractor()

    try:
        rclpy.spin(extractor)
    except KeyboardInterrupt:
        print("interrupt received, closing...")
    finally:
        extractor.close()
        if rclpy.ok():
            extractor.destroy_node()
            rclpy.shutdown()

    print(f"extraction complete. results saved to {OUTPUT_FILENAME}")

if __name__ == '__main__':
    main()
