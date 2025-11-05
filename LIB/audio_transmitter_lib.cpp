#include "audio_transmitter.h"
#include "tone_generator.h"
#include "audio_comm.h"
#include <memory>

namespace AudioComm {

// ============================================================================
// SingleToneTransmitter Implementation
// ============================================================================

class SingleToneTransmitter::Impl {
public:
	bool startTransmitting(uint8_t value, const Config& config) {
		config_ = config;
		currentValue_ = value;

		// Create encoder
		SingleToneConfig encConfig;
		encConfig.minFreq = config.minFreq;
		encConfig.maxFreq = config.maxFreq;
		encConfig.bitsPerTone = config.bitsPerTone;
		encConfig.toneDuration = config.toneDuration;
		encConfig.fadeTime = config.fadeTime;
		encConfig.sampleRate = config.sampleRate;

		SingleToneEncoder encoder(encConfig);
		currentFrequency_ = encoder.encodeValue(value);

		// Configure tone generator
		ToneGenerator::Config toneConfig;
		toneConfig.frequencies = {currentFrequency_};
		toneConfig.duration = config.toneDuration;
		toneConfig.gain = config.gain;
		toneConfig.fadeTime = config.fadeTime;
		toneConfig.sampleRate = config.sampleRate;

		return generator_.start(toneConfig);
	}

	bool startTransmittingAsync(uint8_t value, const Config& config,
								std::function<void()> onComplete) {
		config_ = config;
		currentValue_ = value;

		// Create encoder
		SingleToneConfig encConfig;
		encConfig.minFreq = config.minFreq;
		encConfig.maxFreq = config.maxFreq;
		encConfig.bitsPerTone = config.bitsPerTone;
		encConfig.toneDuration = config.toneDuration;
		encConfig.fadeTime = config.fadeTime;
		encConfig.sampleRate = config.sampleRate;

		SingleToneEncoder encoder(encConfig);
		currentFrequency_ = encoder.encodeValue(value);

		// Configure tone generator
		ToneGenerator::Config toneConfig;
		toneConfig.frequencies = {currentFrequency_};
		toneConfig.duration = config.toneDuration;
		toneConfig.gain = config.gain;
		toneConfig.fadeTime = config.fadeTime;
		toneConfig.sampleRate = config.sampleRate;

		return generator_.startAsync(toneConfig, onComplete);
	}

	void stop() {
		generator_.stop();
	}

	bool isTransmitting() const {
		return generator_.isPlaying();
	}

	double getCurrentFrequency() const {
		return currentFrequency_;
	}

	void waitForCompletion() {
		generator_.waitForCompletion();
	}

private:
	ToneGenerator generator_;
	Config config_;
	uint8_t currentValue_ = 0;
	double currentFrequency_ = 0.0;
};

SingleToneTransmitter::SingleToneTransmitter() : impl_(new Impl()) {}
SingleToneTransmitter::~SingleToneTransmitter() { delete impl_; }

bool SingleToneTransmitter::startTransmitting(uint8_t value, const Config& config) {
	return impl_->startTransmitting(value, config);
}

bool SingleToneTransmitter::startTransmitting(uint8_t value) {
	return impl_->startTransmitting(value, Config());
}

bool SingleToneTransmitter::startTransmittingAsync(uint8_t value, const Config& config,
												   std::function<void()> onComplete) {
	return impl_->startTransmittingAsync(value, config, onComplete);
}

bool SingleToneTransmitter::startTransmittingAsync(uint8_t value,
												   std::function<void()> onComplete) {
	return impl_->startTransmittingAsync(value, Config(), onComplete);
}

void SingleToneTransmitter::stop() {
	impl_->stop();
}

bool SingleToneTransmitter::isTransmitting() const {
	return impl_->isTransmitting();
}

double SingleToneTransmitter::getCurrentFrequency() const {
	return impl_->getCurrentFrequency();
}

void SingleToneTransmitter::waitForCompletion() {
	impl_->waitForCompletion();
}

// ============================================================================
// ChordTransmitter Implementation
// ============================================================================

class ChordTransmitter::Impl {
public:
	bool startTransmitting(uint16_t value, const Config& config) {
		config_ = config;
		currentValue_ = value;

		// Create encoder
		ChordConfig encConfig;
		encConfig.tone1MinFreq = config.tone1MinFreq;
		encConfig.tone1MaxFreq = config.tone1MaxFreq;
		encConfig.tone2MinFreq = config.tone2MinFreq;
		encConfig.tone2MaxFreq = config.tone2MaxFreq;
		encConfig.tone3MinFreq = config.tone3MinFreq;
		encConfig.tone3MaxFreq = config.tone3MaxFreq;
		encConfig.tone4MinFreq = config.tone4MinFreq;
		encConfig.tone4MaxFreq = config.tone4MaxFreq;
		encConfig.bitsPerTone = config.bitsPerTone;
		encConfig.toneDuration = config.toneDuration;
		encConfig.fadeTime = config.fadeTime;
		encConfig.sampleRate = config.sampleRate;

		ChordEncoder encoder(encConfig);
		currentFrequencies_ = encoder.encodeValue(value);

		// Configure tone generator
		ToneGenerator::Config toneConfig;
		toneConfig.frequencies = currentFrequencies_;
		toneConfig.duration = config.toneDuration;
		toneConfig.gain = config.gain;
		toneConfig.fadeTime = config.fadeTime;
		toneConfig.sampleRate = config.sampleRate;

		return generator_.start(toneConfig);
	}

	bool startTransmittingAsync(uint16_t value, const Config& config,
								std::function<void()> onComplete) {
		config_ = config;
		currentValue_ = value;

		// Create encoder
		ChordConfig encConfig;
		encConfig.tone1MinFreq = config.tone1MinFreq;
		encConfig.tone1MaxFreq = config.tone1MaxFreq;
		encConfig.tone2MinFreq = config.tone2MinFreq;
		encConfig.tone2MaxFreq = config.tone2MaxFreq;
		encConfig.tone3MinFreq = config.tone3MinFreq;
		encConfig.tone3MaxFreq = config.tone3MaxFreq;
		encConfig.tone4MinFreq = config.tone4MinFreq;
		encConfig.tone4MaxFreq = config.tone4MaxFreq;
		encConfig.bitsPerTone = config.bitsPerTone;
		encConfig.toneDuration = config.toneDuration;
		encConfig.fadeTime = config.fadeTime;
		encConfig.sampleRate = config.sampleRate;

		ChordEncoder encoder(encConfig);
		currentFrequencies_ = encoder.encodeValue(value);

		// Configure tone generator
		ToneGenerator::Config toneConfig;
		toneConfig.frequencies = currentFrequencies_;
		toneConfig.duration = config.toneDuration;
		toneConfig.gain = config.gain;
		toneConfig.fadeTime = config.fadeTime;
		toneConfig.sampleRate = config.sampleRate;

		return generator_.startAsync(toneConfig, onComplete);
	}

	void stop() {
		generator_.stop();
	}

	bool isTransmitting() const {
		return generator_.isPlaying();
	}

	std::vector<double> getCurrentFrequencies() const {
		return currentFrequencies_;
	}

	void waitForCompletion() {
		generator_.waitForCompletion();
	}

private:
	ToneGenerator generator_;
	Config config_;
	uint16_t currentValue_ = 0;
	std::vector<double> currentFrequencies_;
};

ChordTransmitter::ChordTransmitter() : impl_(new Impl()) {}
ChordTransmitter::~ChordTransmitter() { delete impl_; }

bool ChordTransmitter::startTransmitting(uint16_t value, const Config& config) {
	return impl_->startTransmitting(value, config);
}

bool ChordTransmitter::startTransmitting(uint16_t value) {
	return impl_->startTransmitting(value, Config());
}

bool ChordTransmitter::startTransmittingAsync(uint16_t value, const Config& config,
											  std::function<void()> onComplete) {
	return impl_->startTransmittingAsync(value, config, onComplete);
}

bool ChordTransmitter::startTransmittingAsync(uint16_t value,
											  std::function<void()> onComplete) {
	return impl_->startTransmittingAsync(value, Config(), onComplete);
}

void ChordTransmitter::stop() {
	impl_->stop();
}

bool ChordTransmitter::isTransmitting() const {
	return impl_->isTransmitting();
}

std::vector<double> ChordTransmitter::getCurrentFrequencies() const {
	return impl_->getCurrentFrequencies();
}

void ChordTransmitter::waitForCompletion() {
	impl_->waitForCompletion();
}

} // namespace AudioComm
