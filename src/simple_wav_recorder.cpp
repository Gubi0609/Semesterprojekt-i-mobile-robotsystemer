#include <portaudio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <atomic>
#include <chrono>
#include <iomanip>

// Simple WAV file recorder - records what the microphone hears to a WAV file
// Useful for debugging microphone issues

std::atomic<bool> running(true);

void signalHandler(int signum) {
	std::cout << "\n\nStopping recording...\n";
	running = false;
}

// WAV file header structure
struct WavHeader {
	char riff[4] = {'R', 'I', 'F', 'F'};
	uint32_t fileSize;
	char wave[4] = {'W', 'A', 'V', 'E'};
	char fmt[4] = {'f', 'm', 't', ' '};
	uint32_t fmtSize = 16;
	uint16_t audioFormat = 1; // PCM
	uint16_t numChannels;
	uint32_t sampleRate;
	uint32_t byteRate;
	uint16_t blockAlign;
	uint16_t bitsPerSample;
	char data[4] = {'d', 'a', 't', 'a'};
	uint32_t dataSize;
};

class WavRecorder {
public:
	WavRecorder(int sampleRate = 48000, int channels = 1)
		: sampleRate_(sampleRate)
		, channels_(channels)
		, stream_(nullptr)
	{
	}

	~WavRecorder() {
		stop();
	}

	bool start(const std::string& filename, double duration = 0.0) {
		filename_ = filename;
		duration_ = duration;

		// Initialize PortAudio
		PaError err = Pa_Initialize();
		if (err != paNoError) {
			std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << "\n";
			return false;
		}

		// Open default input device
		PaStreamParameters inputParams;
		inputParams.device = Pa_GetDefaultInputDevice();
		if (inputParams.device == paNoDevice) {
			std::cerr << "No default input device found!\n";
			Pa_Terminate();
			return false;
		}

		const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(inputParams.device);
		std::cout << "Using input device: " << deviceInfo->name << "\n";
		std::cout << "Max input channels: " << deviceInfo->maxInputChannels << "\n";
		std::cout << "Default sample rate: " << deviceInfo->defaultSampleRate << "\n";

		inputParams.channelCount = channels_;
		inputParams.sampleFormat = paInt16;
		inputParams.suggestedLatency = deviceInfo->defaultLowInputLatency;
		inputParams.hostApiSpecificStreamInfo = nullptr;

		// Open stream
		err = Pa_OpenStream(
			&stream_,
			&inputParams,
			nullptr, // no output
			sampleRate_,
			1024, // frames per buffer
			paClipOff,
			nullptr, // no callback (blocking mode)
			nullptr
		);

		if (err != paNoError) {
			std::cerr << "Failed to open stream: " << Pa_GetErrorText(err) << "\n";
			Pa_Terminate();
			return false;
		}

		// Start stream
		err = Pa_StartStream(stream_);
		if (err != paNoError) {
			std::cerr << "Failed to start stream: " << Pa_GetErrorText(err) << "\n";
			Pa_CloseStream(stream_);
			Pa_Terminate();
			return false;
		}

		std::cout << "Recording started...\n";
		std::cout << "Sample rate: " << sampleRate_ << " Hz\n";
		std::cout << "Channels: " << channels_ << "\n";
		if (duration_ > 0) {
			std::cout << "Duration: " << duration_ << " seconds\n";
		} else {
			std::cout << "Duration: Until Ctrl+C\n";
		}
		std::cout << "Press Ctrl+C to stop\n\n";

		// Record audio
		recordToFile();

		return true;
	}

	void stop() {
		if (stream_) {
			Pa_StopStream(stream_);
			Pa_CloseStream(stream_);
			stream_ = nullptr;
		}
		Pa_Terminate();
	}

private:
	void recordToFile() {
		std::vector<int16_t> buffer(1024 * channels_);
		std::vector<int16_t> allSamples;

		auto startTime = std::chrono::steady_clock::now();
		auto lastUpdateTime = startTime;
		int16_t maxAmplitude = 0;
		int64_t totalSamples = 0;

		while (running) {
			// Check duration
			if (duration_ > 0) {
				auto now = std::chrono::steady_clock::now();
				double elapsed = std::chrono::duration<double>(now - startTime).count();
				if (elapsed >= duration_) {
					break;
				}
			}

			// Read audio
			PaError err = Pa_ReadStream(stream_, buffer.data(), buffer.size() / channels_);
			if (err != paNoError && err != paInputOverflowed) {
				std::cerr << "Read error: " << Pa_GetErrorText(err) << "\n";
				break;
			}

			// Append to recording
			allSamples.insert(allSamples.end(), buffer.begin(), buffer.end());

			// Track peak amplitude
			for (int16_t sample : buffer) {
				int16_t abs_sample = (sample < 0) ? -sample : sample;
				if (abs_sample > maxAmplitude) {
					maxAmplitude = abs_sample;
				}
			}

			totalSamples += buffer.size();

			// Print progress every second
			auto now = std::chrono::steady_clock::now();
			double timeSinceUpdate = std::chrono::duration<double>(now - lastUpdateTime).count();
			if (timeSinceUpdate >= 1.0) {
				double elapsed = std::chrono::duration<double>(now - startTime).count();
				double peakPercent = (maxAmplitude / 32768.0) * 100.0;
				std::cout << "[" << std::fixed << std::setprecision(1) << elapsed << "s] "
						  << "Recording... Peak: " << std::setw(5) << std::setprecision(1)
						  << peakPercent << "% | Samples: " << totalSamples << "\n";

				// Warn if signal is too weak
				if (peakPercent < 1.0) {
					std::cout << "  ⚠️  WARNING: Very weak signal! Check microphone volume\n";
				} else if (peakPercent < 5.0) {
					std::cout << "  ⚠️  WARNING: Weak signal. Consider increasing volume\n";
				} else if (peakPercent > 95.0) {
					std::cout << "  ⚠️  WARNING: Signal clipping! Reduce microphone gain\n";
				}

				lastUpdateTime = now;
				maxAmplitude = 0; // Reset for next second
			}
		}

		auto endTime = std::chrono::steady_clock::now();
		double totalTime = std::chrono::duration<double>(endTime - startTime).count();

		std::cout << "\n";
		std::cout << "Recording stopped.\n";
		std::cout << "Total time: " << std::fixed << std::setprecision(2) << totalTime << " seconds\n";
		std::cout << "Total samples: " << allSamples.size() << "\n";
		std::cout << "Writing to file: " << filename_ << "\n";

		// Write WAV file
		writeWavFile(filename_, allSamples);
	}

	void writeWavFile(const std::string& filename, const std::vector<int16_t>& samples) {
		std::ofstream file(filename, std::ios::binary);
		if (!file) {
			std::cerr << "Failed to open output file: " << filename << "\n";
			return;
		}

		// Prepare WAV header
		WavHeader header;
		header.numChannels = channels_;
		header.sampleRate = sampleRate_;
		header.bitsPerSample = 16;
		header.byteRate = sampleRate_ * channels_ * (16 / 8);
		header.blockAlign = channels_ * (16 / 8);
		header.dataSize = samples.size() * sizeof(int16_t);
		header.fileSize = 36 + header.dataSize;

		// Write header
		file.write(reinterpret_cast<const char*>(&header), sizeof(header));

		// Write audio data
		file.write(reinterpret_cast<const char*>(samples.data()), samples.size() * sizeof(int16_t));

		file.close();

		std::cout << "✓ File written successfully!\n";
		std::cout << "\nTo play the recording:\n";
		std::cout << "  aplay " << filename << "\n";
		std::cout << "\nTo analyze the recording:\n";
		std::cout << "  sox " << filename << " -n stat\n";
		std::cout << "  sox " << filename << " -n spectrogram -o spectrogram.png\n";
	}

	int sampleRate_;
	int channels_;
	double duration_;
	std::string filename_;
	PaStream* stream_;
};

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║                    WAV RECORDER - MIC DEBUGGER                     ║\n";
	std::cout << "║   Records what your microphone hears to a WAV file                ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Parse arguments
	std::string filename = "recording.wav";
	double duration = 0.0; // 0 = until Ctrl+C
	int sampleRate = 48000;

	if (argc > 1) {
		filename = argv[1];
	}
	if (argc > 2) {
		duration = std::stod(argv[2]);
	}
	if (argc > 3) {
		sampleRate = std::stoi(argv[3]);
	}

	std::cout << "Usage: " << argv[0] << " [filename] [duration] [samplerate]\n";
	std::cout << "  filename:   Output WAV file (default: recording.wav)\n";
	std::cout << "  duration:   Recording duration in seconds (default: 0 = until Ctrl+C)\n";
	std::cout << "  samplerate: Sample rate in Hz (default: 48000)\n\n";

	// Create and start recorder
	WavRecorder recorder(sampleRate, 1); // Mono recording

	if (!recorder.start(filename, duration)) {
		std::cerr << "Failed to start recording!\n";
		return 1;
	}

	std::cout << "\n✓ Recording complete!\n";
	return 0;
}
