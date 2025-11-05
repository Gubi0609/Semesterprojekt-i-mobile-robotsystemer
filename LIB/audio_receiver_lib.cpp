#include "audio_receiver.h"
#include "frequency_detector.h"
#include "audio_comm.h"
#include <map>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <memory>

namespace AudioComm {

// ============================================================================
// SingleToneReceiver Implementation
// ============================================================================

class SingleToneReceiver::Impl {
public:
	bool startReceiving(const Config& config,
						std::function<void(const Detection&)> onDetection) {
		config_ = config;
		onDetection_ = onDetection;

		// Setup decoder
		SingleToneConfig decConfig;
		decConfig.minFreq = config.minFreq;
		decConfig.maxFreq = config.maxFreq;
		decConfig.bitsPerTone = config.bitsPerTone;
		decConfig.detectionTolerance = config.detectionTolerance;
		decConfig.sampleRate = config.sampleRate;
		decConfig.fftSize = config.fftSize;

		decoder_ = std::make_unique<SingleToneDecoder>(decConfig);

		// Setup frequency detector
		FrequencyDetector::Config detConfig;
		detConfig.sampleRate = static_cast<int>(config.sampleRate);
		detConfig.fftSize = config.fftSize;
		detConfig.numPeaks = 5;
		detConfig.duration = 0.0; // Continuous
		detConfig.bandpassLow = config.minFreq - 500;
		detConfig.bandpassHigh = config.maxFreq + 500;
		detConfig.updateRate = config.updateRate;

		// Detection callback
		auto callback = [this](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
			handleDetection(peaks);
		};

		return detector_.startAsync(detConfig, callback);
	}

	void stop() {
		detector_.stop();
	}

	bool isReceiving() const {
		return detector_.isDetecting();
	}

	bool getLastDetection(Detection& detection) const {
		std::lock_guard<std::mutex> lock(detectionMutex_);
		if (!hasLastDetection_) return false;
		detection = lastDetection_;
		return true;
	}

	void waitForCompletion() {
		detector_.waitForCompletion();
	}

private:
	void handleDetection(const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		if (peaks.empty() || !decoder_) return;

		auto now = std::chrono::steady_clock::now();

		// Find strongest valid peak
		for (const auto& peak : peaks) {
			if (!decoder_->isValidFrequency(peak.frequency)) continue;

			int decodedValue = decoder_->decodeFrequency(peak.frequency);
			if (decodedValue < 0) continue;

			// Check if this is a repeat detection
			double timeSinceLast = std::chrono::duration<double>(
				now - lastDetectionTime_).count();

			if (decodedValue != lastValue_ || timeSinceLast > 0.15) {
				Detection det;
				det.value = static_cast<uint8_t>(decodedValue);
				det.frequency = peak.frequency;
				det.magnitude = peak.magnitude;
				det.detectionCount = 1;

				{
					std::lock_guard<std::mutex> lock(detectionMutex_);
					lastDetection_ = det;
					hasLastDetection_ = true;
				}

				if (onDetection_) {
					onDetection_(det);
				}

				lastValue_ = decodedValue;
				lastDetectionTime_ = now;
			}
			break;
		}
	}

	FrequencyDetector detector_;
	std::unique_ptr<SingleToneDecoder> decoder_;
	Config config_;
	std::function<void(const Detection&)> onDetection_;

	mutable std::mutex detectionMutex_;
	Detection lastDetection_;
	bool hasLastDetection_ = false;

	int lastValue_ = -1;
	std::chrono::steady_clock::time_point lastDetectionTime_;
};

SingleToneReceiver::SingleToneReceiver() : impl_(new Impl()) {}
SingleToneReceiver::~SingleToneReceiver() { delete impl_; }

bool SingleToneReceiver::startReceiving(const Config& config,
										std::function<void(const Detection&)> onDetection) {
	return impl_->startReceiving(config, onDetection);
}

void SingleToneReceiver::stop() {
	impl_->stop();
}

bool SingleToneReceiver::isReceiving() const {
	return impl_->isReceiving();
}

bool SingleToneReceiver::getLastDetection(Detection& detection) const {
	return impl_->getLastDetection(detection);
}

void SingleToneReceiver::waitForCompletion() {
	impl_->waitForCompletion();
}

// ============================================================================
// ChordReceiver Implementation
// ============================================================================

class ChordReceiver::Impl {
public:
	bool startReceiving(const Config& config,
						std::function<void(const Detection&)> onDetection) {
		config_ = config;
		onDetection_ = onDetection;

		// Setup decoder
		ChordConfig decConfig;
		decConfig.tone1MinFreq = config.tone1MinFreq;
		decConfig.tone1MaxFreq = config.tone1MaxFreq;
		decConfig.tone2MinFreq = config.tone2MinFreq;
		decConfig.tone2MaxFreq = config.tone2MaxFreq;
		decConfig.tone3MinFreq = config.tone3MinFreq;
		decConfig.tone3MaxFreq = config.tone3MaxFreq;
		decConfig.tone4MinFreq = config.tone4MinFreq;
		decConfig.tone4MaxFreq = config.tone4MaxFreq;
		decConfig.bitsPerTone = config.bitsPerTone;
		decConfig.detectionTolerance = config.detectionTolerance;
		decConfig.sampleRate = config.sampleRate;
		decConfig.fftSize = config.fftSize;

		decoder_ = std::make_unique<ChordDecoder>(decConfig);

		// Setup frequency detector
		FrequencyDetector::Config detConfig;
		detConfig.sampleRate = static_cast<int>(config.sampleRate);
		detConfig.fftSize = config.fftSize;
		detConfig.numPeaks = 10;
		detConfig.duration = 0.0; // Continuous
		detConfig.bandpassLow = config.tone1MinFreq - 500;
		detConfig.bandpassHigh = config.tone4MaxFreq + 500;
		detConfig.updateRate = config.updateRate;

		// Detection callback
		auto callback = [this](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
			handleDetection(peaks);
		};

		return detector_.startAsync(detConfig, callback);
	}

	void stop() {
		detector_.stop();
	}

	bool isReceiving() const {
		return detector_.isDetecting();
	}

	bool getLastDetection(Detection& detection) const {
		std::lock_guard<std::mutex> lock(detectionMutex_);
		if (!hasLastDetection_) return false;
		detection = lastDetection_;
		return true;
	}

	void waitForCompletion() {
		detector_.waitForCompletion();
	}

private:
	struct ChordCandidate {
		int32_t value;
		int count;
		std::chrono::steady_clock::time_point firstSeen;
		std::chrono::steady_clock::time_point lastSeen;
		std::vector<double> frequencies;
		std::vector<uint8_t> toneValues;
	};

	void handleDetection(const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		if (peaks.empty() || !decoder_) return;

		auto now = std::chrono::steady_clock::now();

		// Identify which tones are present
		std::map<int, int> currentTones;

		for (const auto& peak : peaks) {
			if (!decoder_->isValidFrequency(peak.frequency)) continue;

			auto [toneIndex, toneValue] = decoder_->decodeSingleFrequency(peak.frequency);
			if (toneIndex >= 0 && toneValue >= 0) {
				if (currentTones.find(toneIndex) == currentTones.end()) {
					currentTones[toneIndex] = toneValue;
				}
			}
		}

		// Check if we have all 4 tones
		if (currentTones.size() == 4) {
			// Build frequency vector
			std::vector<double> detectedFreqs(4);
			std::vector<uint8_t> toneValues(4);
			bool hasAll = true;

			for (int i = 0; i < 4; ++i) {
				if (currentTones.find(i) == currentTones.end()) {
					hasAll = false;
					break;
				}

				int toneValue = currentTones[i];
				toneValues[i] = static_cast<uint8_t>(toneValue);

				// Reconstruct frequency
				double minFreq, maxFreq;
				switch(i) {
					case 0: minFreq = config_.tone1MinFreq; maxFreq = config_.tone1MaxFreq; break;
					case 1: minFreq = config_.tone2MinFreq; maxFreq = config_.tone2MaxFreq; break;
					case 2: minFreq = config_.tone3MinFreq; maxFreq = config_.tone3MaxFreq; break;
					case 3: minFreq = config_.tone4MinFreq; maxFreq = config_.tone4MaxFreq; break;
					default: minFreq = maxFreq = 0;
				}
				double freqStep = (maxFreq - minFreq) / 15.0;
				detectedFreqs[i] = minFreq + (toneValue * freqStep);
			}

			if (hasAll) {
				int32_t decodedValue = decoder_->decodeFrequencies(detectedFreqs);

				if (decodedValue >= 0) {
					// Clean up old candidates
					auto it = chordCandidates_.begin();
					while (it != chordCandidates_.end()) {
						double age = std::chrono::duration<double>(
							now - it->second.firstSeen).count();
						if (age > config_.consistencyWindow) {
							it = chordCandidates_.erase(it);
						} else {
							++it;
						}
					}

					// Update or create candidate
					if (chordCandidates_.find(decodedValue) == chordCandidates_.end()) {
						chordCandidates_[decodedValue] = {
							decodedValue, 1, now, now, detectedFreqs, toneValues
						};
					} else {
						chordCandidates_[decodedValue].count++;
						chordCandidates_[decodedValue].lastSeen = now;
						chordCandidates_[decodedValue].frequencies = detectedFreqs;
						chordCandidates_[decodedValue].toneValues = toneValues;
					}

					// Check if confirmed
					auto& candidate = chordCandidates_[decodedValue];
					if (candidate.count >= config_.minDetections) {
						double timeSinceLast = std::chrono::duration<double>(
							now - lastDetectionTime_).count();

						if (decodedValue != lastValue_ || timeSinceLast > 0.5) {
							double detectionTime = std::chrono::duration<double>(
								candidate.lastSeen - candidate.firstSeen).count();

							Detection det;
							det.value = static_cast<uint16_t>(decodedValue);
							det.frequencies = candidate.frequencies;
							det.toneValues = candidate.toneValues;
							det.magnitude = 0.8; // Could calculate average
							det.detectionCount = candidate.count;
							det.detectionTime = detectionTime;

							{
								std::lock_guard<std::mutex> lock(detectionMutex_);
								lastDetection_ = det;
								hasLastDetection_ = true;
							}

							if (onDetection_) {
								onDetection_(det);
							}

							lastValue_ = decodedValue;
							lastDetectionTime_ = now;

							// Reset candidate
							chordCandidates_.erase(decodedValue);
						}
					}
				}
			}
		}
	}

	FrequencyDetector detector_;
	std::unique_ptr<ChordDecoder> decoder_;
	Config config_;
	std::function<void(const Detection&)> onDetection_;

	mutable std::mutex detectionMutex_;
	Detection lastDetection_;
	bool hasLastDetection_ = false;

	int32_t lastValue_ = -1;
	std::chrono::steady_clock::time_point lastDetectionTime_;
	std::map<int32_t, ChordCandidate> chordCandidates_;
};

ChordReceiver::ChordReceiver() : impl_(new Impl()) {}
ChordReceiver::~ChordReceiver() { delete impl_; }

bool ChordReceiver::startReceiving(const Config& config,
									std::function<void(const Detection&)> onDetection) {
	return impl_->startReceiving(config, onDetection);
}

void ChordReceiver::stop() {
	impl_->stop();
}

bool ChordReceiver::isReceiving() const {
	return impl_->isReceiving();
}

bool ChordReceiver::getLastDetection(Detection& detection) const {
	return impl_->getLastDetection(detection);
}

void ChordReceiver::waitForCompletion() {
	impl_->waitForCompletion();
}

} // namespace AudioComm
