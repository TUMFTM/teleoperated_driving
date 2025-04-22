import pyshark
import csv

PCAP_FILEPATH = 'data/trace_operator_1.pcap'
HOST1 = '10.0.0.10'
HOST2 = '10.100.80.2'
OUTPUT_FILENAME = 'protocol_data.csv'

"""
Load captured .pcap file and compute protocol usage percentage.
Filter out traffic ONLY between host1 and host2.
"""
def load_and_filter_packets(pcap_file, host1, host2):

    # the information we want to extract, e.g. amount of bytes
    protocol_data = {'TCP': 0, 'UDP': 0, 'ICMP': 0}

    # load capture
    cap = pyshark.FileCapture(pcap_file, display_filter=f"ip.addr=={host1} && ip.addr=={host2}")

    for packet in cap:
        try:
            packet_size = int(packet.length)
            if 'TCP' in packet:
                protocol_data['TCP'] += packet_size
            elif 'UDP' in packet:
                protocol_data['UDP'] += packet_size
            elif 'ICMP' in packet:
                if int(packet.icmp.type) in [8, 0]:  # types for ECHO_REQUEST and ECHO_REPLY
                    protocol_data['ICMP'] += packet_size
        except AttributeError:
            continue # skip packets that do not match

    cap.close()
    return protocol_data

def save_to_csv(data, output_file):
    with open(output_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Protocol', 'Bytes'])
        for protocol, bytes in data.items():
            writer.writerow([protocol, bytes])

def main():

    protocol_data = load_and_filter_packets(PCAP_FILEPATH, HOST1, HOST2)

    save_to_csv(protocol_data, OUTPUT_FILENAME)
    print(f"data processing complete. results saved to {OUTPUT_FILENAME}")

if __name__ == "__main__":
    main()

