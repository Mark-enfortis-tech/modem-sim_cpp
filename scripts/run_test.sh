#!/bin/bash

# Check if socat is installed
if ! command -v socat &> /dev/null; then
    echo "socat is not installed. Please install it with:"
    echo "sudo apt-get install socat"
    exit 1
fi

# Kill any existing socat processes
echo "Stopping any existing socat processes..."
pkill -f "socat.*vserial" || true
sleep 1

# Create virtual serial port pairs
echo "Creating virtual serial port pairs..."
socat -d -d pty,raw,echo=0,link=/tmp/vserial0 pty,raw,echo=0,link=/tmp/vserial1 &
SOCAT_PID1=$!
echo "Created pair: /tmp/vserial0 <-> /tmp/vserial1"

socat -d -d pty,raw,echo=0,link=/tmp/vserial2 pty,raw,echo=0,link=/tmp/vserial3 &
SOCAT_PID2=$!
echo "Created pair: /tmp/vserial2 <-> /tmp/vserial3"

# Wait for ports to be ready
sleep 1

# Start the simulator in the background
echo "Starting the simulator..."
./plm_simulator -s /tmp/vserial0 -r /tmp/vserial2 &
SIMULATOR_PID=$!
echo "Simulator started with PID $SIMULATOR_PID"

# Wait for simulator to initialize
sleep 1

# Start the receiver in the background
echo "Starting the receiver..."
./test_receiver /tmp/vserial3 &
RECEIVER_PID=$!
echo "Receiver started with PID $RECEIVER_PID"

# Wait for receiver to initialize
sleep 1

# Send a test message
echo "Sending test message..."
./test_sender /tmp/vserial1 "Hello from the power line modem simulator!"

# Wait for a moment to see the results
sleep 2

# Clean up
echo "Cleaning up..."
kill $SIMULATOR_PID $RECEIVER_PID $SOCAT_PID1 $SOCAT_PID2 2>/dev/null
echo "Test completed."
