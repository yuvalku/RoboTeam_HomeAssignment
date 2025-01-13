#!/bin/bash

send_can_drive_msgs() {
    end_time=$((SECONDS + 2))

    while [ $SECONDS -lt $end_time ]; do
        cansend can0 203#01000000
        cansend can0 204#01000000
    done

    cansend can0 203#01000000
    cansend can0 204#01000000
}

send_can_drive_msgs