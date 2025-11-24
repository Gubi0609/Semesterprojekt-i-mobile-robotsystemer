# Automatic Microphone Restart Feature

## Problem

When the microphone is left open for extended periods, audio stream issues can occur:
- Buffer overruns
- Stream drift
- PortAudio/ALSA connection issues
- Gradual degradation of detection quality

## Solution: Idle-Based Auto-Restart

Implemented an automatic restart mechanism that **only triggers during idle periods** to avoid interrupting active communication.

### How It Works

```
Timeline:
0s    - User sends command
0.4s  - Command detected → Activity timestamp updated
5s    - No activity for 5s (still < 30s threshold)
30s   - No activity for 30s → RESTART TRIGGERED
30.2s - Microphone restarted
30.2s - Ready to receive commands again
```

### Key Features

1. **Activity-Based Timing:**
   - Tracks time since last successful detection
   - Only restarts when idle for 30+ seconds
   - Any command received resets the timer

2. **Non-Disruptive:**
   - Never interrupts during active communication
   - Brief restart takes ~200ms
   - Automatic resumption with same configuration

3. **Transparent Operation:**
   - User sees notification when restart occurs
   - No action required from user
   - Commands continue to work normally after restart

## Implementation Details

### Tracking Variables

```cpp
std::chrono::steady_clock::time_point lastActivityTime;
const double RESTART_INTERVAL = 30.0;  // Restart after 30s idle
bool needsRestart = false;
```

### Activity Detection

Every time a command is successfully detected:
```cpp
lastActivityTime = now;  // Reset the idle timer
```

### Restart Logic

Main loop checks every 100ms:
```cpp
auto timeSinceActivity = duration(now - lastActivityTime).count();

if (timeSinceActivity >= 30.0 && !needsRestart) {
    // Stop receiver
    // Wait 200ms
    // Restart with same config
    // Reset timer
}
```

### Restart Process

1. **Stop:** Closes current audio stream (~50-100ms)
2. **Pause:** 200ms delay to ensure clean shutdown
3. **Restart:** Opens new audio stream (~100-200ms)
4. **Resume:** Listening for commands again

**Total time:** ~350-500ms (imperceptible to user when idle)

## Configuration

### Startup Display

The receiver now shows:
```
FFT Size: 16384 (Frequency resolution: 2.93 Hz per bin)
Detection Tolerance: 150.0 Hz
Duplicate Lockout: 0.8 seconds
Auto-restart interval: 30.0 seconds (when idle)
```

### During Restart

When restart occurs:
```
⟳ Idle for 30s - Restarting microphone...
✓ Microphone restarted successfully
Listening for commands...
```

## Behavior Examples

### Example 1: Active Communication (No Restart)
```
Time | Event
-----|-------------------------------------------
0s   | Command received → Timer reset
5s   | Command received → Timer reset
10s  | Command received → Timer reset
15s  | Command received → Timer reset
...  | Commands keep coming, no restart ever triggered
```

### Example 2: Becomes Idle (Restart Triggered)
```
Time | Event
-----|-------------------------------------------
0s   | Command received → Timer reset
2s   | Last command received
32s  | 30s since last command → RESTART
32.4s| Microphone ready again
45s  | Command received → Works normally
```

### Example 3: Restart Interrupted by Command
```
Time | Event
-----|-------------------------------------------
0s   | Command received → Timer reset
29s  | Almost 30s idle...
29.5s| New command received → Timer reset!
...  | No restart, communication continues
```

## Timing Considerations

### Why 30 Seconds?

**Too Short (< 10s):**
- Restarts too frequently
- May interrupt legitimate pauses between commands
- Unnecessary overhead

**Too Long (> 60s):**
- Allows problems to persist
- Less effective at preventing issues
- Defeats the purpose

**30 Seconds (Chosen):**
- ✅ Long enough to avoid interrupting normal use
- ✅ Short enough to prevent extended issues
- ✅ Natural pause length for typical operation
- ✅ Matches human behavior patterns

### Restart Duration (~200-500ms)

**Why Safe During Idle:**
- No transmission expected during 30s idle period
- User has stopped sending commands
- Brief interruption doesn't affect user experience
- Automatic resumption is seamless

**Why NOT Safe During Active Period:**
- 0.8s transmission window
- 500ms restart = 62% of transmission missed
- Would cause command loss
- That's why we only restart when idle!

## Edge Cases Handled

### 1. First Startup
- Timer initialized at startup
- No restart until first 30s idle period

### 2. Rapid Commands
- Each detection resets timer
- Restart never triggered during active use

### 3. Restart Failure
- Error message displayed
- Attempts to continue (may need manual restart)
- User aware of the issue

### 4. Shutdown During Restart
- Ctrl+C handled gracefully
- Clean shutdown regardless of state

### 5. Multiple Restarts
- Each successful restart resets timer
- Pattern can repeat indefinitely
- No limit on number of restarts

## Adjusting the Interval

To change the restart interval:

```cpp
// In protocol_receiver.cpp:
const double RESTART_INTERVAL = 30.0;  // Change this value
```

**Recommended ranges:**
- **Minimum:** 10 seconds (for testing)
- **Typical:** 20-30 seconds (production)
- **Maximum:** 60 seconds (if rarely idle)

## Benefits

✅ **Prevents audio stream degradation** from long-running sessions
✅ **Non-disruptive** - only restarts when idle
✅ **Automatic** - no user intervention required
✅ **Transparent** - user informed of restart
✅ **Reliable** - handles failures gracefully
✅ **Configurable** - easy to adjust interval

## Performance Impact

**During Normal Operation:**
- Check every 100ms: ~1 µs overhead
- Negligible performance impact

**During Restart:**
- 350-500ms total downtime
- Only occurs during idle periods
- No impact on command throughput

## Testing

### Manual Test:
```bash
cd BUILD
./protocol_receiver 1

# Wait 30 seconds without sending commands
# Should see: "⟳ Idle for 30s - Restarting microphone..."

# Send a command after restart
# Should work normally
```

### Stress Test:
```bash
# Send commands continuously for 5 minutes
# Should never restart during active communication

# Stop sending commands
# Should restart after 30s idle
```

## Files Modified

1. **src/protocol_receiver.cpp**
   - Added `lastActivityTime` tracking
   - Added `RESTART_INTERVAL` constant (30s)
   - Added `needsRestart` flag
   - Activity timestamp updated on each detection
   - Restart logic in main loop
   - Refactored callback to be reusable

## Alternative Approaches Considered

### 1. Fixed Interval Restart (Not Implemented)
```cpp
// Restart every 30s regardless of activity
```
**Rejected because:** Would interrupt active communication

### 2. Command Count Restart (Not Implemented)
```cpp
// Restart after every 100 commands
```
**Rejected because:** Doesn't address time-based issues

### 3. Manual Restart Command (Not Implemented)
```cpp
// User sends special "RESTART" command
```
**Rejected because:** Requires user intervention

### 4. Chosen: Idle-Based Restart ✓
**Selected because:** 
- Non-disruptive
- Automatic
- Addresses time-based issues
- Simple implementation

## Known Limitations

⚠️ **Brief interruption during restart:**
- If command arrives during 500ms restart window, it will be missed
- Unlikely due to 30s idle requirement
- Sender should retry if no response

⚠️ **Timer not persistent:**
- Resets on program restart
- Not an issue for typical use

## Future Enhancements

Possible improvements:
1. Adaptive interval based on usage patterns
2. Health monitoring to trigger restart on quality degradation
3. Restart on error detection (CRC failures)
4. Statistics logging (restart count, reasons, timing)

---
*Implemented: 2025-11-13*
*Restart Interval: 30 seconds (when idle)*
*Restart Duration: ~350-500ms*
*Trigger: No activity for 30 seconds*
