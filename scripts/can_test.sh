#!/bin/bash

send_can_drive_msgs() {
    end_time=$((SECONDS + 10))

    while [ $SECONDS -lt $end_time ]; do
        cansend can0 103#01020000
        cansend can0 104#01030000 
        cansend can0 203#015E00
        cansend can0 204#01A2FF
         
    done

    cansend can0 103#01000000
    cansend can0 104#01000000
    cansend can0 203#01000000
    cansend can0 204#01000000
}

send_can_drive_msgs