#!/bin/bash

# TC / qdisc, but with a nice menu for once!

show_menu() {
    echo "Choose an option:"
    echo "1) Configure bandwidth and packet loss"
    echo "2) Show current settings"
    echo "3) Reset settings"
    echo "4) Exit"
}

validate_bandwidth() {
    if [[ $1 =~ ^[0-9]+(kbit|mbit|gbit)$ ]]; then
        return 0
    else
        echo "Invalid bandwidth rate: $1. Please enter a valid rate (e.g., 200mbit)."
        return 1
    fi
}

validate_burst() {
    if [[ $1 =~ ^[0-9]+(kbit|mbit|gbit)$ ]]; then
        return 0
    else
        echo "Invalid burst size: $1. Please enter a valid size (e.g., 100kbit)."
        return 1
    fi
}

validate_latency() {
    if [[ $1 =~ ^[0-9]+ms$ ]]; then
        return 0
    else
        echo "Invalid latency: $1. Please enter a valid latency (e.g., 50ms)."
        return 1
    fi
}

validate_loss() {
    if [[ $1 =~ ^[0-9]+%$ ]]; then
        return 0
    else
        echo "Invalid packet loss: $1. Please enter a valid percentage (e.g., 30%)."
        return 1
    fi
}

configure_bandwidth_and_loss() {
    while true; do
        read -p "Enter bandwidth rate (e.g., 200mbit): " rate
        validate_bandwidth $rate && break
    done
    while true; do
        read -p "Enter burst size (e.g., 100kbit): " burst
        validate_burst $burst && break
    done
    while true; do
        read -p "Enter latency (e.g., 50ms): " latency
        validate_latency $latency && break
    done
    while true; do
        read -p "Enter packet loss percentage (e.g., 30%): " loss
        validate_loss $loss && break
    done

    sudo tc qdisc del dev $interface root 2>/dev/null
    sudo tc qdisc add dev $interface root handle 1: netem delay $latency loss $loss
    sudo tc qdisc add dev $interface parent 1: handle 10: tbf rate $rate burst $burst latency $latency

    if [ $? -eq 0 ]; then
        echo "Configured bandwidth to $rate, burst $burst, latency $latency, and packet loss to $loss"
    else
        echo "Failed to configure settings. Please check your inputs and try again."
    fi
}

show_settings() {
    sudo tc qdisc show dev $interface
    if [ $? -ne 0 ]; then
        echo "Failed to show settings. Please check your network interface."
    fi
}

reset_settings() {
    sudo tc qdisc del dev $interface root
    if [ $? -eq 0 ]; then
        echo "Settings reset to default"
    else
        echo "Failed to reset settings. Please check your network interface."
    fi
}

# main loop
while true; do
    read -p "Enter network interface (e.g., eth0): " interface
    if [ -n "$interface" ]; then
        break
    else
        echo "Please enter a valid interface."
    fi
done

while true; do
    show_menu
    read -p "Enter your choice: " choice
    case $choice in
        1)
            configure_bandwidth_and_loss
            ;;
        2)
            show_settings
            ;;
        3)
            reset_settings
            ;;
        4)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid option. Please try again."
            ;;
    esac
done
