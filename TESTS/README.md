# Test Programs

This directory contains all test and diagnostic programs for the audio communication protocol.

## Frequency Tests
- `frequency_test.cpp` - Tests all 64 frequencies (16 values × 4 tones)
- `frequency_test_single.cpp` - Tests individual single-tone detection
- `frequency_transmitter_test.cpp` - Systematically transmits all frequencies
- `frequency_response_test.cpp` - Tests frequency response
- `frequency_detector.cpp` - Frequency detector test program

## Chord Tests
- `chord_transmitter_test.cpp` - Sends 4-tone chords for testing
- `chord_diagnostic_test.cpp` - Diagnostic tool showing chord detection process
- `chord_receiver.cpp` - Chord receiver test
- `chord_transmitter.cpp` - Chord transmitter test
- `chord_detection_accuracy_test.cpp` - Tests chord detection accuracy
- `chord_to_bits.cpp` - Chord to bits conversion test (in LIB/examples/)

## Protocol Tests
- `protocol_test.cpp` - Protocol layer testing

## Detection Tests
- `detection_count_test.cpp` - Tests detection counting

## Recording Tools
- `simple_wav_recorder.cpp` - Records microphone input to WAV file (useful for debugging)
- `mp3_recorder.cpp` - MP3 recording tool

## Transmitter/Receiver Tests
- `transmitter.cpp` - Basic transmitter test
- `receiver.cpp` - Basic receiver test
- `tones.cpp` - Tone generation test

## Building Tests

Most tests can be built using the Makefile in SRC/:
```bash
cd SRC
make frequency_response_test
make detection_count_test
make chord_accuracy_test
# etc.
```

Or compile manually with:
```bash
g++ -std=c++17 -I../LIB test_name.cpp [required libs] -o test_name -lportaudio -lfftw3
```

## Documentation
- `FREQUENCY_TEST_README.md` - Documentation for frequency testing
- `FFT_RESOLUTION_IMPROVEMENTS.md` - FFT resolution improvements documentation

## Notes
Some test programs may be outdated or redundant. Review needed to identify which tests are still actively used.
