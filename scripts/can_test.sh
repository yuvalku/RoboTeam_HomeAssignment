#!/bin/bash

send_can_drive_msgs() {
    end_time=$((SECONDS + 5))

    while [ $SECONDS -lt $end_time ]; do
        cansend can0 203#015200
        cansend can0 204#01ADFE
    done

    cansend can0 203#01000000
    cansend can0 204#01000000
}

send_can_drive_msgs