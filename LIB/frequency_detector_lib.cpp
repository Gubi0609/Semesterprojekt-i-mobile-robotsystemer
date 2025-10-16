#include "frequency_detector.h"
#include <portaudio.h>
#include <fftw3.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
#include <limits.h>

// Implementation class (PIMPL pattern)
class FrequencyDetector::Impl {
public:
	Impl() {
		PaError err = Pa_Initialize();
		if (err != paNoError) {
			std::cerr << "Pa_Initialize error: " << Pa_GetErrorText(err) << "\n";
			initialized_ = false;
		} else {
			initialized_ = true;
		}
	}

	~Impl() {
		stop();
		cleanup();
		if (initialized_) {
			Pa_Terminate();
		}
	}

	std::vector<FrequencyDetector::FrequencyPeak> start(const FrequencyDetector::Config& config) {
		if (!startAsync(config, nullptr)) {
			return {};
		}

		// Wait for duration or until stopped
		if (config.duration > 0) {
			auto startTime = std::chrono::steady_clock::now();
			while (detecting_.load()) {
				auto elapsed = std::chrono::steady_clock::now() - startTime;
				if (std::chrono::duration<double>(elapsed).count() >= config.duration) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}
			stop();
		} else {
			waitForCompletion();
		}

		return getDetectedFrequencies();
	}

	bool startAsync(const FrequencyDetector::Config& config,
				   std::function<void(const std::vector<FrequencyDetector::FrequencyPeak>&)> callback) {
		if (!initialized_) return false;
		if (detecting_.load()) {
			stop();
		}

		config_ = config;
		callback_ = callback;

		// Allocate FFT buffers
		inputBuffer_.resize(config_.fftSize);
		fftIn_ = fftw_alloc_real(config_.fftSize);
		fftOut_ = fftw_alloc_complex(config_.fftSize / 2 + 1);
		plan_ = fftw_plan_dft_r2c_1d(config_.fftSize, fftIn_, fftOut_, FFTW_ESTIMATE);

		// Initialize window
		window_.resize(config_.fftSize);
		for (int i = 0; i < config_.fftSize; ++i) {
			window_[i] = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (config_.fftSize - 1)));
		}

		// Initialize bandpass filter if needed
		if (config_.bandpassHigh > 0.0) {
			if (config_.bandpassLow <= 0.0) config_.bandpassLow = 20.0;
			if (config_.bandpassHigh > config_.sampleRate / 2.0) {
				config_.bandpassHigh = config_.sampleRate / 2.0;
			}
			useBandpass_ = true;
			initBandpassFilter();
		}

		// Setup PortAudio stream
		PaStreamParameters inParams;
		std::memset(&inParams, 0, sizeof(inParams));
		inParams.device = Pa_GetDefaultInputDevice();
		if (inParams.device == paNoDevice) {
			std::cerr << "No default input device.\n";
			cleanup();
			return false;
		}
		inParams.channelCount = 1;
		inParams.sampleFormat = paFloat32;
		inParams.suggestedLatency = Pa_GetDeviceInfo(inParams.device)->defaultLowInputLatency;
		inParams.hostApiSpecificStreamInfo = nullptr;

		PaError err = Pa_OpenStream(&stream_,
								   &inParams,
								   nullptr,
								   static_cast<double>(config_.sampleRate),
								   config_.fftSize / 4,
								   paClipOff,
								   audioCallback,
								   this);
		if (err != paNoError) {
			std::cerr << "Pa_OpenStream: " << Pa_GetErrorText(err) << "\n";
			cleanup();
			return false;
		}

		err = Pa_StartStream(stream_);
		if (err != paNoError) {
			std::cerr << "Pa_StartStream: " << Pa_GetErrorText(err) << "\n";
			Pa_CloseStream(stream_);
			stream_ = nullptr;
			cleanup();
			return false;
		}

		detecting_.store(true);
		startTime_ = std::chrono::steady_clock::now();

		// Launch processing thread
		if (processingThread_.joinable()) {
			processingThread_.join();
		}
		processingThread_ = std::thread(&Impl::processingLoop, this);

		return true;
	}

	void stop() {
		if (!detecting_.load()) return;

		detecting_.store(false);

		if (processingThread_.joinable()) {
			processingThread_.join();
		}

		if (stream_) {
			Pa_StopStream(stream_);
			Pa_CloseStream(stream_);
			stream_ = nullptr;
		}

		cleanup();
	}

	bool isDetecting() const {
		return detecting_.load();
	}

	std::vector<FrequencyDetector::FrequencyPeak> getDetectedFrequencies() {
		std::lock_guard<std::mutex> lock(peaksMutex_);
		return detectedPeaks_;
	}

	void waitForCompletion() {
		if (config_.duration > 0) {
			auto duration = std::chrono::duration<double>(config_.duration);
			while (detecting_.load()) {
				auto elapsed = std::chrono::steady_clock::now() - startTime_;
				if (elapsed >= duration) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}
			stop();
		} else {
			while (detecting_.load()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}
		}
	}

private:
	static int audioCallback(const void* input,
							void* output,
							unsigned long frameCount,
							const PaStreamCallbackTimeInfo* timeInfo,
							PaStreamCallbackFlags statusFlags,
							void* userData)
	{
		(void)output;
		(void)timeInfo;
		(void)statusFlags;

		Impl* impl = static_cast<Impl*>(userData);
		const float* in = static_cast<const float*>(input);

		if (input == nullptr) return paContinue;
		if (!impl->detecting_.load()) return paComplete;

		impl->processAudio(in, frameCount);
		return paContinue;
	}

	void processAudio(const float* samples, unsigned long frameCount) {
		std::lock_guard<std::mutex> lock(bufferMutex_);

		size_t samplesToShift = inputBuffer_.size() - frameCount;
		if (samplesToShift > 0) {
			std::memmove(inputBuffer_.data(),
						inputBuffer_.data() + frameCount,
						samplesToShift * sizeof(float));
		}

		if (useBandpass_) {
			for (unsigned long i = 0; i < frameCount; ++i) {
				inputBuffer_[samplesToShift + i] = applyBandpassFilter(samples[i]);
			}
		} else {
			std::memcpy(inputBuffer_.data() + samplesToShift,
					   samples,
					   frameCount * sizeof(float));
		}

		newDataAvailable_.store(true);
	}

	void processingLoop() {
		using namespace std::chrono_literals;
		int updateInterval = static_cast<int>(1000.0 / config_.updateRate);

		while (detecting_.load()) {
			// Check duration
			if (config_.duration > 0) {
				auto elapsed = std::chrono::steady_clock::now() - startTime_;
				if (std::chrono::duration<double>(elapsed).count() >= config_.duration) {
					detecting_.store(false);
					break;
				}
			}

			if (newDataAvailable_.load()) {
				performFFT();
				newDataAvailable_.store(false);

				if (callback_) {
					auto peaks = getDetectedFrequencies();
					callback_(peaks);
				}
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(updateInterval));
		}
	}

	void performFFT() {
		std::lock_guard<std::mutex> lock(bufferMutex_);

		for (int i = 0; i < config_.fftSize; ++i) {
			fftIn_[i] = inputBuffer_[i] * window_[i];
		}

		fftw_execute(plan_);

		std::vector<FrequencyDetector::FrequencyPeak> peaks;
		const int numBins = config_.fftSize / 2 + 1;
		const double freqPerBin = static_cast<double>(config_.sampleRate) / config_.fftSize;

		std::vector<double> magnitudes(numBins);
		double maxMag = 0.0;

		for (int i = 0; i < numBins; ++i) {
			double real = fftOut_[i][0];
			double imag = fftOut_[i][1];
			magnitudes[i] = std::sqrt(real * real + imag * imag);
			maxMag = std::max(maxMag, magnitudes[i]);
		}

		const double threshold = maxMag * 0.1;
		const int minBin = static_cast<int>(20.0 / freqPerBin);

		for (int i = minBin + 1; i < numBins - 1; ++i) {
			if (magnitudes[i] > magnitudes[i-1] &&
				magnitudes[i] > magnitudes[i+1] &&
				magnitudes[i] > threshold)
			{
				FrequencyDetector::FrequencyPeak peak;
				peak.frequency = i * freqPerBin;
				peak.magnitude = magnitudes[i] / maxMag;
				peak.magnitudeDB = 20.0 * std::log10(magnitudes[i] + 1e-10);
				peaks.push_back(peak);
			}
		}

		std::sort(peaks.begin(), peaks.end(),
				 [](const FrequencyDetector::FrequencyPeak& a,
					const FrequencyDetector::FrequencyPeak& b) {
					 return a.magnitude > b.magnitude;
				 });

		if (peaks.size() > static_cast<size_t>(config_.numPeaks)) {
			peaks.resize(config_.numPeaks);
		}

		{
			std::lock_guard<std::mutex> peaksLock(peaksMutex_);
			detectedPeaks_ = peaks;
		}
	}

	void initBandpassFilter() {
		double centerFreq = (config_.bandpassLow + config_.bandpassHigh) / 2.0;
		double bandwidth = config_.bandpassHigh - config_.bandpassLow;
		double Q = centerFreq / bandwidth;
		double w0 = 2.0 * M_PI * centerFreq / config_.sampleRate;
		double alpha = std::sin(w0) / (2.0 * Q);

		double a0 = 1.0 + alpha;
		b0_ = alpha / a0;
		b1_ = 0.0;
		b2_ = -alpha / a0;
		a1_ = -2.0 * std::cos(w0) / a0;
		a2_ = (1.0 - alpha) / a0;

		x1_ = x2_ = y1_ = y2_ = 0.0;
	}

	float applyBandpassFilter(float input) {
		double output = b0_ * input + b1_ * x1_ + b2_ * x2_
					   - a1_ * y1_ - a2_ * y2_;
		x2_ = x1_;
		x1_ = input;
		y2_ = y1_;
		y1_ = output;
		return static_cast<float>(output);
	}

	void cleanup() {
		if (plan_) {
			fftw_destroy_plan(plan_);
			plan_ = nullptr;
		}
		if (fftIn_) {
			fftw_free(fftIn_);
			fftIn_ = nullptr;
		}
		if (fftOut_) {
			fftw_free(fftOut_);
			fftOut_ = nullptr;
		}
	}

	bool initialized_ = false;
	std::atomic<bool> detecting_{false};
	FrequencyDetector::Config config_;
	std::function<void(const std::vector<FrequencyDetector::FrequencyPeak>&)> callback_;

	PaStream* stream_ = nullptr;
	fftw_plan plan_ = nullptr;
	double* fftIn_ = nullptr;
	fftw_complex* fftOut_ = nullptr;

	std::vector<double> window_;
	std::vector<float> inputBuffer_;
	std::mutex bufferMutex_;
	std::atomic<bool> newDataAvailable_{false};

	std::vector<FrequencyDetector::FrequencyPeak> detectedPeaks_;
	std::mutex peaksMutex_;

	std::thread processingThread_;
	std::chrono::steady_clock::time_point startTime_;

	bool useBandpass_ = false;
	double b0_, b1_, b2_, a1_, a2_;
	double x1_, x2_, y1_, y2_;
};

// FrequencyDetector wrapper implementation
FrequencyDetector::~FrequencyDetector() {
	delete impl_;
}

std::vector<FrequencyDetector::FrequencyPeak> FrequencyDetector::start(const Config& config) {
	if (!impl_) impl_ = new Impl();
	return impl_->start(config);
}

bool FrequencyDetector::startAsync(const Config& config,
								   std::function<void(const std::vector<FrequencyPeak>&)> callback) {
	if (!impl_) impl_ = new Impl();
	return impl_->startAsync(config, callback);
}

void FrequencyDetector::stop() {
	if (impl_) impl_->stop();
}

bool FrequencyDetector::isDetecting() const {
	return impl_ ? impl_->isDetecting() : false;
}

std::vector<FrequencyDetector::FrequencyPeak> FrequencyDetector::getDetectedFrequencies() {
	return impl_ ? impl_->getDetectedFrequencies() : std::vector<FrequencyPeak>();
}

void FrequencyDetector::waitForCompletion() {
	if (impl_) impl_->waitForCompletion();
}
