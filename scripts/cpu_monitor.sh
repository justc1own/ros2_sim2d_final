#!/bin/bash

# Скрипт профилирования для Главы 3.3
# Замеряет потребление CPU и RAM узлом ros2_control_node (controller_manager).

LOG_FILE="/home/dev/ws/src/sim2d_hardware_interface/scripts/cpu_profile.log"
echo "Timestamp(s), CPU_Usage(%), RAM_Usage(%)" > $LOG_FILE

echo "Starting CPU profiler for ros2_control_node (controller_manager)..."
echo "Logging output to $LOG_FILE (Press Ctrl+C to stop)"

# Wait until the node starts
PID=""
while [ -z "$PID" ]; do
    PID=$(pgrep -f "ros2_control_node")
    if [ -z "$PID" ]; then
        sleep 1
    fi
done

echo "Found ros2_control_node with PID: $PID"

START_TIME=$(date +%s)

while true; do
    # check if process is still running
    if ! kill -0 $PID 2>/dev/null; then
        echo "Process $PID terminated. Exiting profiler."
        break
    fi

    # Read CPU and MEM usage using ps (e.g., ' 0.5  1.2')
    STATS=$(ps -p $PID -o %cpu,%mem | tail -n 1)
    
    # Trim leading whitespace and replace multiple spaces with comma
    STATS_FORMATTED=$(echo $STATS | awk '{print $1 "," $2}')
    
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - START_TIME))

    echo "$ELAPSED, $STATS_FORMATTED" >> $LOG_FILE
    sleep 1
done
