# FFT Resolution Improvements for Chord Detection

## Problem
Certain chords were not being detected correctly or were being misdetected. This is likely due to insufficient frequency resolution in the FFT analysis.

## Root Cause: FFT Frequency Resolution

The frequency resolution of an FFT is determined by:

```
Frequency Resolution = Sample Rate / FFT Size
```

### Previous Configuration:
- **Sample Rate**: 48,000 Hz
- **FFT Size**: 8,192 samples
- **Resolution**: 48,000 / 8,192 ≈ **5.86 Hz per bin**

This means frequencies closer than ~5.86 Hz cannot be distinguished reliably.

### Our Chord System:
- Each tone encodes 4 bits (16 possible values)
- Tone spacing: 200 Hz between adjacent values
- 4 frequency bands:
  - Band 1: 5000-8000 Hz (200 Hz steps)
  - Band 2: 8500-11500 Hz (200 Hz steps)
  - Band 3: 12000-15000 Hz (200 Hz steps)
  - Band 4: 15500-18500 Hz (200 Hz steps)

While 200 Hz spacing should be sufficient with 5.86 Hz resolution, **interference effects** and **spectral leakage** can cause issues, especially when multiple tones are played simultaneously.

## Solution: Increase FFT Size

### New Configuration Options:
1. **8,192 samples** (original) - 5.86 Hz/bin
2. **16,384 samples** (recommended) - **2.93 Hz/bin** ✓
3. **32,768 samples** (very high) - 1.46 Hz/bin

Doubling the FFT size to 16,384 samples provides **twice the frequency resolution**, making it much easier to distinguish between closely-spaced tones and reducing interference effects.

## Changes Made

### 1. Updated protocol_receiver.cpp
Added higher FFT size configuration:

```cpp
// INCREASED FFT SIZE for better frequency resolution
// 16384 samples @ 48kHz = ~2.93 Hz resolution (vs 5.86 Hz with 8192)
recvConfig.fftSize = 16384;
```

Also added diagnostic output showing the frequency resolution being used.

### 2. Created New Test Program: chord_detection_accuracy_test.cpp
A comprehensive test program to identify problematic chords:

**Features:**
- Tests individual chords systematically
- Configurable FFT size (8192, 16384, or 32768)
- Three test modes:
  1. Quick test (16 protocol commands)
  2. Sample test (256 random chords)
  3. Full sweep (all 65536 possible chords)
- Detailed reporting:
  - Lists missed chords with their frequencies
  - Lists misdetected chords (wrong value received)
  - Shows frequency differences for errors
  - Calculates detection accuracy percentage
- CSV export for analysis in spreadsheet software

**Usage:**
```bash
cd BUILD
./chord_accuracy_test     # Interactive mode
./chord_accuracy_test 2   # FFT size 16384 (recommended)
./chord_accuracy_test 2 1 # FFT 16384, Quick test mode
```

**Output Files:**
- `BUILD/chord_accuracy_fft16384.csv` - Detailed results for each chord tested

### 3. Updated Makefile
Added build rule for the new test program:

```makefile
chord_accuracy_test: Test\ programs/chord_detection_accuracy_test.cpp ...
```

Build with:
```bash
cd SRC
make chord_accuracy_test
```

## Trade-offs

### Benefits of Larger FFT:
✓ Better frequency resolution (can distinguish tones more accurately)
✓ Reduced spectral leakage
✓ More reliable detection of closely-spaced frequencies
✓ Better handling of harmonics and interference

### Costs of Larger FFT:
✗ Slightly higher CPU usage (2x more samples to process)
✗ Longer minimum observation window needed (more samples required)
✗ Slightly increased memory usage

**Verdict**: The trade-offs are worth it for improved accuracy. Modern processors handle 16K FFTs easily.

## Expected Results

With 16,384 FFT size:
- Should see **significant improvement** in chord detection accuracy
- Problematic chords should now be detected reliably
- Misdetection rate should drop substantially

## Testing Procedure

1. **Compile updated programs:**
   ```bash
   cd SRC
   make protocol_receiver
   make chord_accuracy_test
   ```

2. **Run accuracy test to identify problems:**
   ```bash
   cd BUILD
   ./chord_accuracy_test 2 1  # FFT 16384, quick test
   ```

3. **Check results:**
   - Console shows summary of detection accuracy
   - CSV file contains detailed per-chord results
   - Look for "MISSED CHORDS" and "WRONG VALUE" sections

4. **If problems persist:**
   - Try FFT size 32768 (option 3)
   - Check CSV for patterns in problematic frequencies
   - May need to adjust tone spacing or frequency bands

## Alternative Solutions (if issues persist)

If increasing FFT size doesn't fully resolve the issue:

1. **Increase tone spacing** (e.g., 300 Hz instead of 200 Hz)
   - Reduces data density but improves reliability
   
2. **Adjust frequency bands** to avoid interference regions
   - Some frequency ranges may have more environmental noise
   
3. **Implement windowing functions** (e.g., Hann window)
   - Reduces spectral leakage
   - Already may be implemented in FFTW library
   
4. **Increase transmission power** (amplitude)
   - Improves signal-to-noise ratio
   - Already configurable in ChordTransmitter

5. **Add filtering** (bandpass for each tone band)
   - Reduces out-of-band noise
   - More complex implementation

## Files Modified

1. `src/protocol_receiver.cpp` - Added FFT size configuration
2. `SRC/Test programs/chord_detection_accuracy_test.cpp` - New test program (created)
3. `SRC/Makefile` - Added build rule for new test program

## Next Steps

1. Run `chord_accuracy_test` with your current setup
2. Review which chords are problematic
3. Share results if issues persist
4. May need to iterate on frequency band configuration

---
*Created: 2025-11-13*
*Protocol Version: Command Protocol v1.0*
