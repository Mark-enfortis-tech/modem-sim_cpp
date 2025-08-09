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

echo "Virtual serial ports are ready."
echo ""
echo "PLM Simulator should use:"
echo "  Sender port: /tmp/vserial0"
echo "  Receiver port: /tmp/vserial2"
echo ""
echo "Connect sender to: /tmp/vserial1"
echo "Connect receiver to: /tmp/vserial3"
echo ""
echo "Press Ctrl+C to stop the virtual ports"

# Wait for Ctrl+C
trap "echo 'Stopping virtual ports...'; kill $SOCAT_PID1 $SOCAT_PID2 2>/dev/null; exit 0" INT TERM
wait
