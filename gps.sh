#!/bin/bash

# Run u-center in the background
wine ~/.wine/drive_c/Program\ Files\ \(x86\)/u-blox/u-center_v24.10/u-center.exe &

# Create symbolic link in the background
ln -sf /dev/tty_Ardusimple ~/.wine/dosdevices/com1 &

# Log NTRIP file save in the background
echo "NTRIP file saved" > ~/gps_ws/logs/ntrip_logs &

# Open gps_publisher_realtime.py in vim in the background
vim ~/autonomous_tow_truck/src/gps_publisher/gps_publisher/gps_publisher_realtime.py &

# Wait for all background processes to finish
wait

