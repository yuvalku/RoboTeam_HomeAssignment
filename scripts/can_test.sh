#!/bin/bash

send_can_drive_msgs() {
    for i in {1..2}; do

        cansend can0 203#016300
        cansend can0 204#019DFF
        sleep 1

        cansend can0 203#01000000
        cansend can0 204#01000000
        sleep 1


    done

    cansend can0 203#01000000
    cansend can0 204#01000000
}

send_tigr_drive_msgs() {
    echo "Starting TIGR drive message sending for 5 seconds..."
    start_time=$(date +%s)  # Get current time in seconds

    while [ $(($(date +%s) - start_time)) -lt 5 ]; do
        # Send left wheel command (+50000)
        cansend can0 67C#2305200946010000

        # Send right wheel command (-50000)
        cansend can0 67C#2305200CBAFEFFFF

        sleep 0.5  # small delay to avoid flooding the CAN bus
    done

    echo "Sending stop commands to wheels..."
    # After 5 seconds, send STOP to both wheels
    cansend can0 67C#2305200900000000
    cansend can0 67C#2305200C00000000
}

#send_can_drive_msgs
send_tigr_drive_msgs