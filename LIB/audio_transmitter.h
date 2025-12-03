#ifndef AUDIO_TRANSMITTER_H
#define AUDIO_TRANSMITTER_H

#include <cstdint>
#include <vector>
#include <functional>

// Audio transmitter library for single-tone and chord communication

namespace AudioComm {

// Forward declarations
class SingleToneEncoder;
class ChordEncoder;

// ============================================================================
// Single Tone Transmitter
// ============================================================================

class SingleToneTransmitter {
public:
	struct Config {
		double minFreq = 5000.0;
		double maxFreq = 8000.0;
		int bitsPerTone = 4;
		double toneDuration = 0.0;    // 0 = infinite
		double fadeTime = 0.05;
		double sampleRate = 48000.0;
		double gain = 0.5;
	};

	SingleToneTransmitter();
	~SingleToneTransmitter();

	// Start transmitting a 4-bit value (0-15)
	// Returns true on success
	bool startTransmitting(uint8_t value, const Config& config);
	bool startTransmitting(uint8_t value); // Use default config

	// Start transmitting asynchronously
	// Callback is called when transmission completes (if duration > 0)
	bool startTransmittingAsync(uint8_t value,
								const Config& config,
								std::function<void()> onComplete = nullptr);
	bool startTransmittingAsync(uint8_t value,
								std::function<void()> onComplete = nullptr); // Use default config

	// Stop transmission
	void stop();

	// Check if currently transmitting
	bool isTransmitting() const;

	// Get the frequency being transmitted
	double getCurrentFrequency() const;

	// Wait for transmission to complete
	void waitForCompletion();

private:
	class Impl;
	Impl* impl_ = nullptr;
};

// ============================================================================
// Chord Transmitter (4 tones = 16 bits)
// ============================================================================

class ChordTransmitter {
public:
	struct Config {
		double tone1MinFreq = 4500.0;
		double tone1MaxFreq = 7000.0;
		double tone2MinFreq = 7500.0;
		double tone2MaxFreq = 10000.0;
		double tone3MinFreq = 10500.0;
		double tone3MaxFreq = 13000.0;
		double tone4MinFreq = 13500.0;  // Under 18 kHz mic limit
		double tone4MaxFreq = 16000.0;
		int bitsPerTone = 4;
		double toneDuration = 0.0;    // 0 = infinite
		double fadeTime = 0.05;
		double sampleRate = 48000.0;
		double gain = 0.4;            // Lower gain for multiple tones
	};

	ChordTransmitter();
	~ChordTransmitter();

	// Start transmitting a 16-bit value (0-65535)
	// Returns true on success
	bool startTransmitting(uint16_t value, const Config& config);
	bool startTransmitting(uint16_t value); // Use default config

	// Start transmitting asynchronously
	// Callback is called when transmission completes (if duration > 0)
	bool startTransmittingAsync(uint16_t value,
								const Config& config,
								std::function<void()> onComplete = nullptr);
	bool startTransmittingAsync(uint16_t value,
								std::function<void()> onComplete = nullptr); // Use default config

	// Stop transmission
	void stop();

	// Check if currently transmitting
	bool isTransmitting() const;

	// Get the frequencies being transmitted
	std::vector<double> getCurrentFrequencies() const;

	// Wait for transmission to complete
	void waitForCompletion();

private:
	class Impl;
	Impl* impl_ = nullptr;
};

} // namespace AudioComm

#endif // AUDIO_TRANSMITTER_H
