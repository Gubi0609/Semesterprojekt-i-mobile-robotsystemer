# Audio Communication Library

Library for audio-based data transmission between devices.

## Quick Start

### Build
```bash
cd ../SRC
make audio_comm
```

### Test
```bash
cd ../BUILD

# Terminal 1 - Single tone (4-bit)
./receiver

# Terminal 2
./transmitter 7

# OR use chord mode (16-bit)
# Terminal 1
./chord_receiver

# Terminal 2
./chord_transmitter 1234
```

## Documentation

📚 **Full documentation** is available in the project wiki.

The documentation source files are in the `DOCS/` folder (not in git).

### Key docs:
- **Getting Started** - System overview and quick start
- **API Reference** - Complete library API documentation
- **Chord Mode Guide** - 16-bit communication details
- **Single Tone Guide** - 4-bit communication details

## Library Files

```
LIB/
├── audio_comm.h/cpp              # Core encoding/decoding
├── audio_transmitter.h/lib.cpp   # Transmitter API
├── audio_receiver.h/lib.cpp      # Receiver API
├── tone_generator.h/lib.cpp      # Audio output
├── frequency_detector.h/lib.cpp  # Audio input
├── transmitter.cpp               # CLI tool
├── receiver.cpp                  # CLI tool
├── chord_transmitter.cpp         # CLI tool
└── chord_receiver.cpp            # CLI tool
```

## Using the Library

```cpp
#include "audio_transmitter.h"
#include "audio_receiver.h"

// Transmit 16-bit value
AudioComm::ChordTransmitter tx;
tx.startTransmitting(1234);

// Receive with callback
AudioComm::ChordReceiver rx;
rx.startReceiving({}, [](const auto& det) {
	std::cout << "Received: " << det.value << "\n";
});
```

See the wiki for complete API reference and examples.

## Dependencies

- PortAudio
- FFTW3
- pthread

## Building Your App

```makefile
OBJS = tone_generator_lib.o frequency_detector_lib.o \
	   audio_comm_lib.o audio_transmitter_lib.o audio_receiver_lib.o
LIBS = -lportaudio -lfftw3 -lpthread

myapp: myapp.cpp $(OBJS)
	g++ -std=c++17 -o myapp myapp.cpp $(OBJS) $(LIBS)
```
