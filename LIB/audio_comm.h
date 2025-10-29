#ifndef AUDIO_COMM_H
#define AUDIO_COMM_H

#include <vector>
#include <cstdint>

namespace AudioComm {

// Single-tone communication configuration
struct SingleToneConfig {
	double minFreq = 5000.0;      // Minimum frequency in Hz (5 kHz)
	double maxFreq = 8000.0;      // Maximum frequency in Hz (8 kHz)
	int bitsPerTone = 4;          // 4 bits = 16 possible values (0-15)
	double toneDuration = 0.0;    // Duration of each tone in seconds (0 = infinite)
	double fadeTime = 0.05;       // Fade in/out time in seconds
	double sampleRate = 48000.0;  // Audio sample rate
	int fftSize = 4096;           // FFT size for detection
	double detectionTolerance = 100.0; // Frequency tolerance in Hz
};

// Multi-tone (chord) communication configuration
struct ChordConfig {
	// Frequency ranges for each of 4 tones
	double tone1MinFreq = 5000.0;   // Tone 1: 5-8 kHz
	double tone1MaxFreq = 8000.0;
	double tone2MinFreq = 8500.0;   // Tone 2: 8.5-11.5 kHz
	double tone2MaxFreq = 11500.0;
	double tone3MinFreq = 12000.0;  // Tone 3: 12-15 kHz
	double tone3MaxFreq = 15000.0;
	double tone4MinFreq = 15500.0;  // Tone 4: 15.5-18.5 kHz
	double tone4MaxFreq = 18500.0;

	int bitsPerTone = 4;            // 4 bits per tone = 16 bits total
	double toneDuration = 0.0;      // Duration (0 = infinite)
	double fadeTime = 0.05;         // Fade in/out time in seconds
	double sampleRate = 48000.0;    // Audio sample rate
	int fftSize = 8192;             // Larger FFT for better resolution
	double detectionTolerance = 100.0; // Frequency tolerance in Hz
};

// Encoder: Convert data to frequencies
class SingleToneEncoder {
public:
	explicit SingleToneEncoder(const SingleToneConfig& config = SingleToneConfig());

	// Encode a 4-bit value (0-15) to frequency
	double encodeValue(uint8_t value);

	// Get the frequency step between consecutive values
	double getFrequencyStep() const;

	// Get number of possible values (2^bitsPerTone)
	int getNumValues() const;

private:
	SingleToneConfig config_;
	double freqStep_;
};

// Decoder: Convert detected frequency to data
class SingleToneDecoder {
public:
	explicit SingleToneDecoder(const SingleToneConfig& config = SingleToneConfig());

	// Decode a detected frequency to a 4-bit value (0-15)
	// Returns -1 if frequency is out of range or invalid
	int decodeFrequency(double frequency);

	// Check if a frequency is within valid range
	bool isValidFrequency(double frequency) const;

private:
	SingleToneConfig config_;
	double freqStep_;
};

// ============================================================================
// Multi-Tone (Chord) Communication
// ============================================================================

// Encoder: Convert 16-bit value to 4 frequencies (chord)
class ChordEncoder {
public:
	explicit ChordEncoder(const ChordConfig& config = ChordConfig());

	// Encode a 16-bit value (0-65535) to 4 frequencies
	std::vector<double> encodeValue(uint16_t value) const;

	// Get the 4-bit value for a specific tone (0-3)
	uint8_t getToneValue(uint16_t value, int toneIndex) const;

	// Get frequency for a specific tone and 4-bit value
	double getFrequency(int toneIndex, uint8_t toneValue) const;

private:
	ChordConfig config_;
	std::vector<double> freqSteps_; // Frequency step for each tone
};

// Decoder: Convert detected frequencies to 16-bit value
class ChordDecoder {
public:
	explicit ChordDecoder(const ChordConfig& config = ChordConfig());

	// Decode 4 detected frequencies to a 16-bit value
	// Returns -1 if decoding fails
	int32_t decodeFrequencies(const std::vector<double>& frequencies);

	// Decode a single frequency to determine which tone it is and its value
	// Returns {toneIndex (0-3), toneValue (0-15)} or {-1, -1} if invalid
	std::pair<int, int> decodeSingleFrequency(double frequency);

	// Check if a frequency is valid for any tone
	bool isValidFrequency(double frequency) const;

private:
	ChordConfig config_;
	std::vector<double> freqSteps_; // Frequency step for each tone

	struct ToneRange {
		double minFreq;
		double maxFreq;
		double freqStep;
	};
	std::vector<ToneRange> toneRanges_;
};

} // namespace AudioComm

#endif // AUDIO_COMM_H
