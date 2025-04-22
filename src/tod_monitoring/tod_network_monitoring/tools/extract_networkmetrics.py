import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import csv

from tod_network_monitoring_msgs.msg import NetworkMetrics # ensure this is available in $PYTHONPATH

TOPIC_NAME = '/Vehicle/NetworkMonitoring/NetworkMetrics'
OUTPUT_FILENAME = 'metrics_output.csv'

"""
Subscribes to a topic upon initialization.
Ensure message type is imported correctly!
After destruction (e.g. Ctrl+C), contents are written to a csv file.
"""
class DataExtractor(Node):
    def __init__(self):
        super().__init__('data_extractor')
        self.subscription = self.create_subscription(
            NetworkMetrics,
            TOPIC_NAME,
            self.message_callback,
            qos_profile_sensor_data) # somehow this QoS settings works quite well, but i dont know why :)
        self.csv_file = open(OUTPUT_FILENAME, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Latency', 'Link Quality', 'RX Bitrate', 'TX Bitrate', 'RX Packets', 'TX Packets'])

    """
    Ensure the fields in the message match the definition
    """
    def message_callback(self, msg):
        timestamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        latency = msg.latency
        link_quality = msg.link_quality
        rx_bitrate_mbps = msg.rx_bitrate_mbps
        tx_bitrate_mbps = msg.tx_bitrate_mbps
        rx_packets_s = msg.rx_packets_s
        tx_packets_s = msg.tx_packets_s

        self.csv_writer.writerow([timestamp, latency, link_quality, rx_bitrate_mbps, tx_bitrate_mbps, rx_packets_s, tx_packets_s])
        self.get_logger().info(f"wrote to csv: Timestamp: {timestamp}, Latency: {latency}, Link Quality: {link_quality}, RX Bitrate: {rx_bitrate_mbps}, TX Bitrate: {tx_bitrate_mbps}, RX Packets: {rx_packets_s}, TX Packets: {tx_packets_s}")

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
