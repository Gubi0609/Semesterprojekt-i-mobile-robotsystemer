# Raspberry Pi Debugging Guide

Quick reference for debugging on the Raspberry Pi over SSH.

---

## 🔌 SSH Connection

```bash
# Connect to Pi
ssh pi@<ip-address>

# Or if you have hostname configured
ssh pi@turtlebot.local
```

---

## 📦 Quick Build on Pi

### Option 1: Using Build Script
```bash
cd ~/Semesterprojekt-i-mobile-robotsystemer
./build_pi.sh
```

### Option 2: Manual CMake
```bash
cd ~/Semesterprojekt-i-mobile-robotsystemer
rm -rf build && mkdir build && cd build
cmake .. -DBUILD_DESKTOP=OFF -DBUILD_PI=ON
make -j4  # Pi 3 has 4 cores
```

### Option 3: One-liner Compile
```bash
# Protocol receiver
cd ~/Semesterprojekt-i-mobile-robotsystemer
g++ -std=c++17 -Wall -Wextra -O2 -I../LIB -I../INCLUDE \
	src/protocol_receiver.cpp SRC/command_protocol.cpp SRC/CRC.cpp \
	LIB/audio_receiver_lib.cpp LIB/audio_comm.cpp LIB/frequency_detector_lib.cpp \
	-o BUILD/protocol_receiver -lportaudio -lfftw3 -lm -lpthread
```

---

## 🎤 Audio Device Testing

### List Audio Devices
```bash
# List PortAudio devices
pactl list sources short

# Or using arecord
arecord -l

# Test microphone
arecord -d 5 -f cd test.wav
aplay test.wav
```

### Set Default Microphone
```bash
# Find device name
pactl list sources short

# Set as default
pactl set-default-source <device-name>
```

---

## 🔍 Running & Debugging Protocol Receiver

### Basic Run
```bash
cd BUILD
./protocol_receiver
```

### With Debug Output
```bash
# If program supports verbose flag
./protocol_receiver --verbose

# Or redirect stderr to file
./protocol_receiver 2> debug.log
```

### Run in Background
```bash
./protocol_receiver &
# View output
tail -f nohup.out

# Stop
killall protocol_receiver
```

### Run with gdb
```bash
# Build with debug symbols first
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_PI=ON
make

# Run with gdb
cd ../BUILD
gdb ./protocol_receiver

# In gdb:
(gdb) run
(gdb) backtrace  # if it crashes
(gdb) quit
```

---

## 📊 Monitoring Performance

### CPU Usage
```bash
top
# Press 'P' to sort by CPU
# Press 'q' to quit
```

### Process Info
```bash
# Find process
ps aux | grep protocol_receiver

# Detailed process info
top -p $(pgrep protocol_receiver)
```

### Audio Performance
```bash
# Check PortAudio latency
./protocol_receiver 2>&1 | grep -i latency

# Monitor audio drops
dmesg | grep -i audio
```

---

## 🐛 Common Issues & Fixes

### Issue: "PortAudio not found"
```bash
sudo apt-get update
sudo apt-get install libportaudio2 portaudio19-dev
```

### Issue: "FFTW3 not found"
```bash
sudo apt-get install libfftw3-dev
```

### Issue: "Permission denied" on audio device
```bash
# Add user to audio group
sudo usermod -a -G audio $USER
# Log out and back in for changes to take effect
```

### Issue: "No default audio input"
```bash
# List devices
arecord -l

# Set default in ALSA config
sudo nano /etc/asound.conf
# Add:
# pcm.!default {
#     type hw
#     card 1
# }
```

### Issue: High CPU usage
```bash
# Check if FFTW is using optimization
# Recompile with -O3 flag
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PI=ON
make
```

### Issue: Audio crackling/dropouts
```bash
# Increase audio buffer size in code or config
# Check CPU usage (should be < 80%)
# Reduce FFT resolution if needed
```

---

## 🔧 Live Debugging Techniques

### Add Debug Prints
```cpp
// In protocol_receiver.cpp, add:
#include <iostream>
std::cout << "[DEBUG] Received chord: " << chordValue << std::endl;
```

### Check if Program is Running
```bash
ps aux | grep protocol_receiver
pgrep -a protocol_receiver
```

### Monitor Output in Real-time
```bash
# Run and monitor
./protocol_receiver 2>&1 | tee debug.log

# Or in separate terminals:
# Terminal 1:
./protocol_receiver 2> /tmp/debug.log

# Terminal 2:
tail -f /tmp/debug.log
```

### Test Audio Pipeline
```bash
# Record while receiver is running
arecord -d 10 -f cd test_while_receiving.wav &
./protocol_receiver

# Check recording
aplay test_while_receiving.wav
```

---

## 📡 Network Debugging

### Check Pi is Reachable
```bash
ping <pi-ip-address>
```

### Check SSH Connection
```bash
ssh -v pi@<pi-ip-address>  # Verbose mode
```

### File Transfer
```bash
# Copy file TO Pi
scp file.txt pi@<ip>:~/

# Copy file FROM Pi
scp pi@<ip>:~/BUILD/protocol_receiver ./

# Copy directory
scp -r pi@<ip>:~/logs ./
```

---

## 🧪 Testing Protocol Communication

### Test Sender from Desktop
```bash
# On desktop
cd BUILD
./protocol_sender
```

### Test Receiver on Pi
```bash
# On Pi
cd BUILD
./protocol_receiver
```

### Verify Commands
```bash
# On Pi, add debug output to see received commands
./protocol_receiver 2>&1 | grep "Command:"
```

---

## 📝 Logging Best Practices

### Create Log File
```bash
# Run with timestamped log
./protocol_receiver 2>&1 | while read line; do echo "$(date '+%Y-%m-%d %H:%M:%S') $line"; done | tee protocol_$(date +%s).log
```

### Rotate Logs
```bash
# Keep last 5 logs
ls -t protocol_*.log | tail -n +6 | xargs rm -f
```

### Analyze Logs
```bash
# Count errors
grep -i error protocol.log | wc -l

# Find crashes
grep -i "segmentation\|crash\|fatal" protocol.log

# Extract timing info
grep "Latency\|Time\|Duration" protocol.log
```

---

## 🔄 Quick Restart

### Restart Script
Create `restart_receiver.sh`:
```bash
#!/bin/bash
killall protocol_receiver 2>/dev/null
sleep 1
cd ~/Semesterprojekt-i-mobile-robotsystemer/BUILD
./protocol_receiver &
echo "Protocol receiver restarted"
```

Make executable:
```bash
chmod +x restart_receiver.sh
```

---

## 💾 Backup Current State

```bash
# On Pi, backup before making changes
tar -czf backup_$(date +%s).tar.gz \
	~/Semesterprojekt-i-mobile-robotsystemer/src/ \
	~/Semesterprojekt-i-mobile-robotsystemer/BUILD/protocol_receiver

# Or just source
tar -czf src_backup_$(date +%s).tar.gz src/
```

---

## 🚀 Quick Commands Reference

```bash
# Build
./build_pi.sh

# Run
cd BUILD && ./protocol_receiver

# Run in background
cd BUILD && nohup ./protocol_receiver > receiver.log 2>&1 &

# Check if running
pgrep protocol_receiver

# Stop
killall protocol_receiver

# View logs
tail -f BUILD/receiver.log

# Monitor CPU
top -p $(pgrep protocol_receiver)

# Test audio
arecord -d 3 -f cd test.wav && aplay test.wav

# Clean and rebuild
rm -rf build BUILD && ./build_pi.sh
```

---

## 🆘 Emergency Recovery

### Program Won't Stop
```bash
killall -9 protocol_receiver
```

### Audio Device Locked
```bash
# Kill all audio processes
sudo killall pulseaudio
pulseaudio --start
```

### System Slow/Hanging
```bash
# Check processes
top
# Kill offending process
kill -9 <PID>
```

### Can't SSH In
```bash
# From desktop, try:
ssh -o ConnectTimeout=10 pi@<ip>

# If no response, may need physical access to Pi
# Check power, network cable, etc.
```

---

## 📚 Useful Files to Check

```bash
# System logs
sudo journalctl -xe

# Audio errors
dmesg | grep -i audio

# USB device errors (if using USB mic)
dmesg | grep -i usb

# Application crash reports
ls -lh /var/crash/
```

---

## 🎯 Debugging Checklist

- [ ] Can SSH into Pi?
- [ ] Is program compiled without errors?
- [ ] Are dependencies installed?
- [ ] Is audio device detected?
- [ ] Is microphone permissions OK?
- [ ] Is program running? (check with `pgrep`)
- [ ] Are there error messages in output?
- [ ] Is CPU usage reasonable? (< 80%)
- [ ] Can record audio with arecord?
- [ ] Is sender working on desktop?
- [ ] Are both on same WiFi network?
- [ ] Is audio loud enough?

---

**Good luck with debugging! 🚀**

For more details, see:
- [BUILD_GUIDE.md](BUILD_GUIDE.md) - Build instructions
- [DOCS/guides/CRC_USAGE_GUIDE.md](DOCS/guides/CRC_USAGE_GUIDE.md) - Protocol usage
- [src/README.md](src/README.md) - Pi source code info
