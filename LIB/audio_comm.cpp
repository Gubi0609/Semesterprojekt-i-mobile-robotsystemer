#include "audio_comm.h"
#include <cmath>
#include <algorithm>

namespace AudioComm {

// ============================================================================
// SingleToneEncoder Implementation
// ============================================================================

SingleToneEncoder::SingleToneEncoder(const SingleToneConfig& config)
	: config_(config)
{
	// Calculate frequency step
	// For 4 bits we have 16 values (0-15), so we need 15 steps
	int numSteps = (1 << config_.bitsPerTone) - 1; // 2^4 - 1 = 15
	freqStep_ = (config_.maxFreq - config_.minFreq) / numSteps;
}

double SingleToneEncoder::encodeValue(uint8_t value) {
	// Clamp value to valid range
	int numValues = (1 << config_.bitsPerTone);
	if (value >= numValues) {
		value = numValues - 1;
	}

	// Calculate frequency: minFreq + (value * step)
	return config_.minFreq + (value * freqStep_);
}

double SingleToneEncoder::getFrequencyStep() const {
	return freqStep_;
}

int SingleToneEncoder::getNumValues() const {
	return (1 << config_.bitsPerTone);
}

// ============================================================================
// SingleToneDecoder Implementation
// ============================================================================

SingleToneDecoder::SingleToneDecoder(const SingleToneConfig& config)
	: config_(config)
{
	// Calculate frequency step (same as encoder)
	int numSteps = (1 << config_.bitsPerTone) - 1;
	freqStep_ = (config_.maxFreq - config_.minFreq) / numSteps;
}

int SingleToneDecoder::decodeFrequency(double frequency) {
	// Check if frequency is in valid range (with tolerance)
	if (frequency < config_.minFreq - config_.detectionTolerance ||
		frequency > config_.maxFreq + config_.detectionTolerance) {
		return -1; // Out of range
	}

	// Calculate which value this frequency corresponds to
	double offset = frequency - config_.minFreq;
	int value = static_cast<int>(std::round(offset / freqStep_));

	// Clamp to valid range
	int numValues = (1 << config_.bitsPerTone);
	value = std::clamp(value, 0, numValues - 1);

	// Verify the decoded frequency is close enough to the original
	double expectedFreq = config_.minFreq + (value * freqStep_);
	if (std::abs(frequency - expectedFreq) > config_.detectionTolerance) {
		return -1; // Too far from expected frequency
	}

	return value;
}

bool SingleToneDecoder::isValidFrequency(double frequency) const {
	return (frequency >= config_.minFreq - config_.detectionTolerance &&
			frequency <= config_.maxFreq + config_.detectionTolerance);
}

// ============================================================================
// ChordEncoder Implementation
// ============================================================================

ChordEncoder::ChordEncoder(const ChordConfig& config)
	: config_(config)
{
	// Calculate frequency step for each tone
	int numSteps = (1 << config_.bitsPerTone) - 1; // 15 steps for 16 values

	freqSteps_.resize(4);
	freqSteps_[0] = (config_.tone1MaxFreq - config_.tone1MinFreq) / numSteps;
	freqSteps_[1] = (config_.tone2MaxFreq - config_.tone2MinFreq) / numSteps;
	freqSteps_[2] = (config_.tone3MaxFreq - config_.tone3MinFreq) / numSteps;
	freqSteps_[3] = (config_.tone4MaxFreq - config_.tone4MinFreq) / numSteps;
}

std::vector<double> ChordEncoder::encodeValue(uint16_t value) const {
	std::vector<double> frequencies(4);

	// Extract 4 bits for each tone
	// Tone 1: bits 0-3 (LSB)
	// Tone 2: bits 4-7
	// Tone 3: bits 8-11
	// Tone 4: bits 12-15 (MSB)

	for (int i = 0; i < 4; ++i) {
		uint8_t toneValue = getToneValue(value, i);
		frequencies[i] = getFrequency(i, toneValue);
	}

	return frequencies;
}

uint8_t ChordEncoder::getToneValue(uint16_t value, int toneIndex) const {
	// Extract 4 bits for the specified tone
	int shift = toneIndex * 4;
	return (value >> shift) & 0x0F;
}

double ChordEncoder::getFrequency(int toneIndex, uint8_t toneValue) const {
	// Clamp tone value to 0-15
	if (toneValue > 15) toneValue = 15;

	double minFreq, freqStep;

	switch (toneIndex) {
		case 0:
			minFreq = config_.tone1MinFreq;
			freqStep = freqSteps_[0];
			break;
		case 1:
			minFreq = config_.tone2MinFreq;
			freqStep = freqSteps_[1];
			break;
		case 2:
			minFreq = config_.tone3MinFreq;
			freqStep = freqSteps_[2];
			break;
		case 3:
			minFreq = config_.tone4MinFreq;
			freqStep = freqSteps_[3];
			break;
		default:
			return 0.0;
	}

	return minFreq + (toneValue * freqStep);
}

// ============================================================================
// ChordDecoder Implementation
// ============================================================================

ChordDecoder::ChordDecoder(const ChordConfig& config)
	: config_(config)
{
	// Calculate frequency step for each tone
	int numSteps = (1 << config_.bitsPerTone) - 1;

	freqSteps_.resize(4);
	freqSteps_[0] = (config_.tone1MaxFreq - config_.tone1MinFreq) / numSteps;
	freqSteps_[1] = (config_.tone2MaxFreq - config_.tone2MinFreq) / numSteps;
	freqSteps_[2] = (config_.tone3MaxFreq - config_.tone3MinFreq) / numSteps;
	freqSteps_[3] = (config_.tone4MaxFreq - config_.tone4MinFreq) / numSteps;

	// Setup tone ranges
	toneRanges_.resize(4);
	toneRanges_[0] = {config_.tone1MinFreq, config_.tone1MaxFreq, freqSteps_[0]};
	toneRanges_[1] = {config_.tone2MinFreq, config_.tone2MaxFreq, freqSteps_[1]};
	toneRanges_[2] = {config_.tone3MinFreq, config_.tone3MaxFreq, freqSteps_[2]};
	toneRanges_[3] = {config_.tone4MinFreq, config_.tone4MaxFreq, freqSteps_[3]};
}

int32_t ChordDecoder::decodeFrequencies(const std::vector<double>& frequencies) {
	if (frequencies.size() != 4) return -1;

	uint16_t result = 0;

	// Decode each frequency and combine into 16-bit value
	for (size_t i = 0; i < 4; ++i) {
		auto [toneIndex, toneValue] = decodeSingleFrequency(frequencies[i]);

		if (toneIndex < 0 || toneValue < 0) return -1;
		if (toneIndex != static_cast<int>(i)) return -1; // Wrong tone order

		// Place the 4-bit value in the correct position
		result |= (toneValue << (i * 4));
	}

	return result;
}

std::pair<int, int> ChordDecoder::decodeSingleFrequency(double frequency) {
	// Determine which tone this frequency belongs to
	for (size_t toneIndex = 0; toneIndex < toneRanges_.size(); ++toneIndex) {
		const auto& range = toneRanges_[toneIndex];

		if (frequency >= range.minFreq - config_.detectionTolerance &&
			frequency <= range.maxFreq + config_.detectionTolerance) {

			// Calculate which value this frequency represents
			double offset = frequency - range.minFreq;
			int value = static_cast<int>(std::round(offset / range.freqStep));
			value = std::clamp(value, 0, 15);

			// Verify the decoded frequency is close enough
			double expectedFreq = range.minFreq + (value * range.freqStep);
			if (std::abs(frequency - expectedFreq) <= config_.detectionTolerance) {
				return {static_cast<int>(toneIndex), value};
			}
		}
	}

	return {-1, -1}; // Not found
}

bool ChordDecoder::isValidFrequency(double frequency) const {
	for (const auto& range : toneRanges_) {
		if (frequency >= range.minFreq - config_.detectionTolerance &&
			frequency <= range.maxFreq + config_.detectionTolerance) {
			return true;
		}
	}
	return false;
}

} // namespace AudioComm
