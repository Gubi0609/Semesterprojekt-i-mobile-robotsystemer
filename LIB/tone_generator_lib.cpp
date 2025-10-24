#include "tone_generator.h"
#include <portaudio.h>
#include <cmath>
#include <climits>
#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>

// Implementation class (PIMPL pattern)
class ToneGenerator::Impl {
public:
	struct SynthData {
		std::vector<double> freq;
		std::vector<double> phase;
		double sampleRate = 48000.0;
		double amp = 0.8;
		long long totalFrames = 0;
		long long framesDone = 0;
		long long rampFrames = 0;
		int numChannels = 2;
		std::atomic<bool> shouldStop{false};
	};

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
		if (initialized_) {
			Pa_Terminate();
		}
	}

	bool start(const ToneGenerator::Config& config) {
		if (!initialized_) return false;
		if (playing_.load()) {
			stop();
		}

		// Setup synth data
		data_.freq = config.frequencies;
		data_.phase.assign(config.frequencies.size(), 0.0);
		data_.sampleRate = config.sampleRate;
		data_.amp = config.gain;
		data_.numChannels = config.channels;
		data_.totalFrames = (config.duration > 0)
			? static_cast<long long>(config.duration * config.sampleRate)
			: LLONG_MAX;
		data_.framesDone = 0;
		data_.rampFrames = std::max(1LL, static_cast<long long>(config.fadeTime * config.sampleRate));
		data_.shouldStop.store(false);

		// Setup PortAudio stream
		PaStreamParameters outParams;
		outParams.device = Pa_GetDefaultOutputDevice();
		if (outParams.device == paNoDevice) {
			std::cerr << "No default output device.\n";
			return false;
		}
		outParams.channelCount = data_.numChannels;
		outParams.sampleFormat = paFloat32;
		outParams.suggestedLatency = Pa_GetDeviceInfo(outParams.device)->defaultLowOutputLatency;
		outParams.hostApiSpecificStreamInfo = nullptr;

		PaError err = Pa_OpenStream(&stream_,
								   nullptr,
								   &outParams,
								   data_.sampleRate,
								   256,
								   paClipOff,
								   paCallback,
								   &data_);
		if (err != paNoError) {
			std::cerr << "Pa_OpenStream error: " << Pa_GetErrorText(err) << "\n";
			return false;
		}

		err = Pa_StartStream(stream_);
		if (err != paNoError) {
			std::cerr << "Pa_StartStream error: " << Pa_GetErrorText(err) << "\n";
			Pa_CloseStream(stream_);
			stream_ = nullptr;
			return false;
		}

		playing_.store(true);
		return true;
	}

	bool startAsync(const ToneGenerator::Config& config,
				   std::function<void()> onComplete) {
		if (!start(config)) return false;

		// Launch monitoring thread
		if (asyncThread_.joinable()) {
			asyncThread_.join();
		}

		asyncThread_ = std::thread([this, onComplete]() {
			waitForCompletion();
			if (onComplete) {
				onComplete();
			}
		});

		return true;
	}

	void stop() {
		if (!playing_.load()) return;

		data_.shouldStop.store(true);

		if (stream_) {
			Pa_StopStream(stream_);
			Pa_CloseStream(stream_);
			stream_ = nullptr;
		}

		playing_.store(false);
	}

	bool isPlaying() const {
		return playing_.load();
	}

	double getPlaybackPosition() const {
		return static_cast<double>(data_.framesDone) / data_.sampleRate;
	}

	void waitForCompletion() {
		while (playing_.load() && Pa_IsStreamActive(stream_) == 1) {
			Pa_Sleep(50);
		}
		stop();
	}

private:
	static int paCallback(const void* input,
						 void* output,
						 unsigned long frameCount,
						 const PaStreamCallbackTimeInfo* timeInfo,
						 PaStreamCallbackFlags statusFlags,
						 void* userData)
	{
		(void)input;
		(void)timeInfo;
		(void)statusFlags;

		SynthData* data = static_cast<SynthData*>(userData);
		float* out = static_cast<float*>(output);

		if (data->shouldStop.load()) {
			return paComplete;
		}

		const size_t N = data->freq.size();
		const double sr = data->sampleRate;
		const double norm = (N > 0) ? (1.0 / static_cast<double>(N)) : 0.0;

		for (unsigned long i = 0; i < frameCount; ++i) {
			if (data->framesDone >= data->totalFrames) {
				for (int ch = 0; ch < data->numChannels; ++ch) *out++ = 0.0f;
				continue;
			}

			double sample = 0.0;
			for (size_t k = 0; k < N; ++k) {
				sample += std::sin(data->phase[k]);
				double inc = (2.0 * M_PI * data->freq[k]) / sr;
				data->phase[k] += inc;
				if (data->phase[k] >= 2.0 * M_PI) data->phase[k] -= 2.0 * M_PI;
			}
			sample *= norm;

			// Fade in/out
			long long idx = data->framesDone;
			double env = 1.0;
			if (data->rampFrames > 0) {
				if (idx < data->rampFrames) {
					env = static_cast<double>(idx) / static_cast<double>(data->rampFrames);
				} else if (idx > data->totalFrames - data->rampFrames) {
					env = static_cast<double>(data->totalFrames - idx)
						/ static_cast<double>(data->rampFrames);
				}
				env = std::clamp(env, 0.0, 1.0);
			}

			float outSample = static_cast<float>(sample * data->amp * env);

			for (int ch = 0; ch < data->numChannels; ++ch) {
				*out++ = outSample;
			}

			data->framesDone++;
		}

		if (data->framesDone >= data->totalFrames) return paComplete;
		return paContinue;
	}

	bool initialized_ = false;
	std::atomic<bool> playing_{false};
	PaStream* stream_ = nullptr;
	SynthData data_;
	std::thread asyncThread_;
};

// ToneGenerator wrapper implementation
ToneGenerator::~ToneGenerator() {
	delete impl_;
}

bool ToneGenerator::start(const Config& config) {
	if (!impl_) impl_ = new Impl();
	return impl_->start(config);
}

bool ToneGenerator::startAsync(const Config& config,
							   std::function<void()> onComplete) {
	if (!impl_) impl_ = new Impl();
	return impl_->startAsync(config, onComplete);
}

void ToneGenerator::stop() {
	if (impl_) impl_->stop();
}

bool ToneGenerator::isPlaying() const {
	return impl_ ? impl_->isPlaying() : false;
}

double ToneGenerator::getPlaybackPosition() const {
	return impl_ ? impl_->getPlaybackPosition() : 0.0;
}

void ToneGenerator::waitForCompletion() {
	if (impl_) impl_->waitForCompletion();
}
