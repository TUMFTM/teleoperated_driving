import pandas as pd
import matplotlib.pyplot as plt

INPUT_FILEPATH = 'protocol_data.csv'

def load_data(csv_file):
    return pd.read_csv(csv_file)

def plot_protocol_usage(data):

    # calculate percentage usage of each protocol
    total_bytes = data['Bytes'].sum()
    data['Percentage'] = data['Bytes'] / total_bytes * 100

    # set colors for each protocol
    colors = {'TCP': '#93BAF0', 'UDP': '#FB9E60', 'ICMP': '#D8D5D5'}

    # plot
    plt.figure(figsize=(7, 4))
    bars = plt.bar(data['Protocol'], data['Percentage'], color=[colors[proto] for proto in data['Protocol']])
    plt.xlabel('Protocol')
    plt.ylabel('Percentage of Total Bytes (%)')
    plt.title('Protocol Usage Distribution')
    plt.yscale('log')  # use log scale
    plt.grid(axis='y', linestyle='--', alpha=0.7)

    # add value labels on top of each bar
    for bar in bars:
        yval = bar.get_height()
        if yval < 1:  # Adjust label position
            label_position = 10
        else:
            label_position = yval + yval * 0.05  # add 5% of the height to position the label
        plt.text(bar.get_x() + bar.get_width()/2, label_position, f'{yval:.2f}%', ha='center', va='bottom')

    plt.tight_layout() # ensure tight layout so that it looks nicer

    # save the plot, preferrably as .pdf
    plt.savefig('protocol_usage_percentage.pdf')

def main():
    data = load_data(INPUT_FILEPATH)
    plot_protocol_usage(data)

if __name__ == "__main__":
    main()
