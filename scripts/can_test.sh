#!/bin/bash

send_can_drive_msgs() {
    for i in {1..2}; do

        cansend can0 203#016300
        cansend can0 204#019DFF
        sleep 5

        cansend can0 203#01000000
        cansend can0 204#01000000
        sleep 1

        cansend can0 204#016300
        cansend can0 203#019DFF
        sleep 5
    done

    cansend can0 203#01000000
    cansend can0 204#01000000
}

send_can_drive_msgs