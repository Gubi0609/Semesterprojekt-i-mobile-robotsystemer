#ifndef AUDIO_RECEIVER_H
#define AUDIO_RECEIVER_H

#include <cstdint>
#include <vector>
#include <functional>

// Audio receiver library for single-tone and chord communication

namespace AudioComm {

// Forward declarations
class SingleToneDecoder;
class ChordDecoder;

// ============================================================================
// Single Tone Receiver
// ============================================================================

class SingleToneReceiver {
public:
	struct Config {
		double minFreq = 5000.0;
		double maxFreq = 8000.0;
		int bitsPerTone = 4;
		double sampleRate = 48000.0;
		int fftSize = 4096;
		double detectionTolerance = 100.0;
		double updateRate = 10.0;
	};

	struct Detection {
		uint8_t value;           // Decoded 4-bit value (0-15)
		double frequency;        // Detected frequency in Hz
		double magnitude;        // Signal strength (0.0-1.0)
		int detectionCount;      // Number of times detected (for consistency)
	};

	SingleToneReceiver();
	~SingleToneReceiver();

	// Start receiving with callback for each detected value
	// Returns true on success
	bool startReceiving(const Config& config,
						std::function<void(const Detection&)> onDetection);

	// Stop receiving
	void stop();

	// Check if currently receiving
	bool isReceiving() const;

	// Get the most recent detection (if any)
	bool getLastDetection(Detection& detection) const;

	// Wait for receiving to complete (if duration was set)
	void waitForCompletion();

private:
	class Impl;
	Impl* impl_ = nullptr;
};

// ============================================================================
// Chord Receiver (4 tones = 16 bits)
// ============================================================================

class ChordReceiver {
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
		double sampleRate = 48000.0;
		int fftSize = 8192;
		double detectionTolerance = 100.0;
		double updateRate = 10.0;

		// Consistency checking
		int minDetections = 2;          // Require N consistent detections
		double consistencyWindow = 1.0; // Within N seconds
	};

	struct Detection {
		uint16_t value;                 // Decoded 16-bit value (0-65535)
		std::vector<double> frequencies; // All 4 detected frequencies
		std::vector<uint8_t> toneValues; // 4-bit value for each tone
		double magnitude;                // Average signal strength
		int detectionCount;              // Number of times detected
		double detectionTime;            // Time over which detected (seconds)
	};

	ChordReceiver();
	~ChordReceiver();

	// Start receiving with callback for each confirmed chord
	// Returns true on success
	bool startReceiving(const Config& config,
						std::function<void(const Detection&)> onDetection);

	// Stop receiving
	void stop();

	// Check if currently receiving
	bool isReceiving() const;

	// Get the most recent detection (if any)
	bool getLastDetection(Detection& detection) const;

	// Wait for receiving to complete (if duration was set)
	void waitForCompletion();

private:
	class Impl;
	Impl* impl_ = nullptr;
};

} // namespace AudioComm

#endif // AUDIO_RECEIVER_H
