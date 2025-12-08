#include <portaudio.h>
#include <fftw3.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <thread>
#include <mutex>

// Real-time frequency detector using FFT
// Captures audio from microphone and identifies dominant frequencies
// Compile: g++ -o frequency_detector frequency_detector.cpp -lportaudio -lfftw3 -lm -lpthread

class FrequencyDetector {
public:
	struct FrequencyPeak {
		double frequency;    // Hz
		double magnitude;    // normalized magnitude (0.0 - 1.0)
		double magnitudeDB;  // decibels
	};

	FrequencyDetector(int sampleRate = 48000, int fftSize = 4096, int numPeaks = 5,
					 double lowFreq = 0.0, double highFreq = 0.0)
		: sampleRate_(sampleRate)
		, fftSize_(fftSize)
		, numPeaks_(numPeaks)
		, bandpassLow_(lowFreq)
		, bandpassHigh_(highFreq)
	{
		// Allocate FFT buffers
		inputBuffer_.resize(fftSize_);
		fftIn_ = fftw_alloc_real(fftSize_);
		fftOut_ = fftw_alloc_complex(fftSize_ / 2 + 1);

		// Create FFT plan (FFTW_MEASURE for optimal performance)
		plan_ = fftw_plan_dft_r2c_1d(fftSize_, fftIn_, fftOut_, FFTW_ESTIMATE);

		// Initialize Hann window for better frequency resolution
		window_.resize(fftSize_);
		for (int i = 0; i < fftSize_; ++i) {
			window_[i] = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (fftSize_ - 1)));
		}

		// Initialize bandpass filter if specified
		if (bandpassHigh_ > 0.0) {
			if (bandpassLow_ <= 0.0) bandpassLow_ = 20.0;  // Default low cutoff
			if (bandpassHigh_ > sampleRate_ / 2.0) {
				bandpassHigh_ = sampleRate_ / 2.0;  // Nyquist limit
			}
			useBandpass_ = true;
			initBandpassFilter();
		}
	}

	~FrequencyDetector() {
		stop();
		if (plan_) fftw_destroy_plan(plan_);
		if (fftIn_) fftw_free(fftIn_);
		if (fftOut_) fftw_free(fftOut_);
	}

	bool start() {
		if (running_.load()) {
			std::cerr << "Detector already running.\n";
			return false;
		}

		PaError err = Pa_Initialize();
		if (err != paNoError) {
			std::cerr << "Pa_Initialize: " << Pa_GetErrorText(err) << "\n";
			return false;
		}

		// Configure input stream
		PaStreamParameters inParams;
		std::memset(&inParams, 0, sizeof(inParams));
		inParams.device = Pa_GetDefaultInputDevice();

		if (inParams.device == paNoDevice) {
			std::cerr << "No default input device.\n";
			Pa_Terminate();
			return false;
		}

		inParams.channelCount = 1;  // mono
		inParams.sampleFormat = paFloat32;
		inParams.suggestedLatency = Pa_GetDeviceInfo(inParams.device)->defaultLowInputLatency;
		inParams.hostApiSpecificStreamInfo = nullptr;

		// Open stream with callback
		err = Pa_OpenStream(&stream_,
						   &inParams,
						   nullptr,
						   static_cast<double>(sampleRate_),
						   fftSize_ / 4,  // process quarter-size chunks for overlap
						   paClipOff,
						   audioCallback,
						   this);

		if (err != paNoError) {
			std::cerr << "Pa_OpenStream: " << Pa_GetErrorText(err) << "\n";
			Pa_Terminate();
			return false;
		}

		err = Pa_StartStream(stream_);
		if (err != paNoError) {
			std::cerr << "Pa_StartStream: " << Pa_GetErrorText(err) << "\n";
			Pa_CloseStream(stream_);
			Pa_Terminate();
			return false;
		}

		running_.store(true);
		processingThread_ = std::thread(&FrequencyDetector::displayLoop, this);

		return true;
	}

	void stop() {
		bool expected = true;
		if (!running_.compare_exchange_strong(expected, false)) {
			return;
		}

		if (processingThread_.joinable()) {
			processingThread_.join();
		}

		if (stream_) {
			Pa_StopStream(stream_);
			Pa_CloseStream(stream_);
			stream_ = nullptr;
		}

		Pa_Terminate();
	}

	bool isRunning() const {
		return running_.load();
	}

	std::vector<FrequencyPeak> getDetectedFrequencies() {
		std::lock_guard<std::mutex> lock(peaksMutex_);
		return detectedPeaks_;
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

		FrequencyDetector* detector = static_cast<FrequencyDetector*>(userData);
		const float* in = static_cast<const float*>(input);

		if (input == nullptr) return paContinue;

		detector->processAudio(in, frameCount);
		return paContinue;
	}

	void processAudio(const float* samples, unsigned long frameCount) {
		std::lock_guard<std::mutex> lock(bufferMutex_);

		// Shift existing samples and add new ones
		size_t samplesToShift = inputBuffer_.size() - frameCount;
		if (samplesToShift > 0) {
			std::memmove(inputBuffer_.data(),
						inputBuffer_.data() + frameCount,
						samplesToShift * sizeof(float));
		}

		// Apply bandpass filter if enabled, otherwise copy directly
		if (useBandpass_) {
			for (unsigned long i = 0; i < frameCount; ++i) {
				inputBuffer_[samplesToShift + i] = applyBandpassFilter(samples[i]);
			}
		} else {
			std::memcpy(inputBuffer_.data() + samplesToShift,
					   samples,
					   frameCount * sizeof(float));
		}

		// Trigger FFT processing
		newDataAvailable_.store(true);
	}

	void performFFT() {
		std::lock_guard<std::mutex> lock(bufferMutex_);

		// Apply window function and copy to FFT input
		for (int i = 0; i < fftSize_; ++i) {
			fftIn_[i] = inputBuffer_[i] * window_[i];
		}

		// Perform FFT
		fftw_execute(plan_);

		// Find peaks in frequency domain
		std::vector<FrequencyPeak> peaks;
		const int numBins = fftSize_ / 2 + 1;
		const double freqPerBin = static_cast<double>(sampleRate_) / fftSize_;

		// Calculate magnitudes
		std::vector<double> magnitudes(numBins);
		double maxMag = 0.0;

		for (int i = 0; i < numBins; ++i) {
			double real = fftOut_[i][0];
			double imag = fftOut_[i][1];
			magnitudes[i] = std::sqrt(real * real + imag * imag);
			maxMag = std::max(maxMag, magnitudes[i]);
		}

		// Find peaks (local maxima with significant magnitude)
		const double threshold = maxMag * 0.1;  // 10% of max

		// Set frequency range for peak detection
		int minBin = static_cast<int>(20.0 / freqPerBin);  // ignore below 20 Hz
		int maxBin = numBins - 1;

		// If bandpass filter is enabled, only look within that frequency range
		if (useBandpass_ && bandpassLow_ > 0 && bandpassHigh_ > 0) {
			minBin = std::max(minBin, static_cast<int>(bandpassLow_ / freqPerBin));
			maxBin = std::min(maxBin, static_cast<int>(bandpassHigh_ / freqPerBin));
		}

		for (int i = minBin + 1; i < maxBin - 1; ++i) {
			// Check if this is a local maximum
			if (magnitudes[i] > magnitudes[i-1] &&
				magnitudes[i] > magnitudes[i+1] &&
				magnitudes[i] > threshold)
			{
				FrequencyPeak peak;
				peak.frequency = i * freqPerBin;
				peak.magnitude = magnitudes[i] / maxMag;
				peak.magnitudeDB = 20.0 * std::log10(magnitudes[i] + 1e-10);
				peaks.push_back(peak);
			}
		}

		// Sort by magnitude (descending)
		std::sort(peaks.begin(), peaks.end(),
				 [](const FrequencyPeak& a, const FrequencyPeak& b) {
					 return a.magnitude > b.magnitude;
				 });

		// Keep only top N peaks
		if (peaks.size() > static_cast<size_t>(numPeaks_)) {
			peaks.resize(numPeaks_);
		}

		// Update detected peaks
		{
			std::lock_guard<std::mutex> peaksLock(peaksMutex_);
			detectedPeaks_ = peaks;
		}
	}

	void displayLoop() {
		using namespace std::chrono_literals;

		std::cout << "\n========================================\n";
		std::cout << "  Real-time Frequency Detector\n";
		std::cout << "  Sample Rate: " << sampleRate_ << " Hz\n";
		std::cout << "  FFT Size: " << fftSize_ << "\n";
		std::cout << "  Frequency Resolution: "
				  << (static_cast<double>(sampleRate_) / fftSize_) << " Hz\n";
		if (useBandpass_) {
			std::cout << "  Bandpass Filter: " << bandpassLow_ << " - "
					  << bandpassHigh_ << " Hz\n";
		}
		std::cout << "========================================\n";
		std::cout << "Press Ctrl+C to stop...\n\n";

		while (running_.load()) {
			if (newDataAvailable_.load()) {
				performFFT();
				newDataAvailable_.store(false);

				// Display results
				auto peaks = getDetectedFrequencies();

				// Clear previous output (move cursor up)
				if (!peaks.empty()) {
					std::cout << "\r\033[K";  // Clear line

					std::cout << "Dominant Frequencies: ";
					for (size_t i = 0; i < peaks.size(); ++i) {
						std::cout << std::fixed << std::setprecision(1)
								  << peaks[i].frequency << " Hz ("
								  << std::setprecision(0)
								  << (peaks[i].magnitude * 100.0) << "%)";
						if (i < peaks.size() - 1) std::cout << " | ";
					}
					std::cout << std::flush;
				}
			}

			std::this_thread::sleep_for(50ms);
		}

		std::cout << "\n\nDetector stopped.\n";
	}

	// Biquad bandpass filter implementation (2nd order)
	void initBandpassFilter() {
		// Calculate filter coefficients for bandpass filter
		double centerFreq = (bandpassLow_ + bandpassHigh_) / 2.0;
		double bandwidth = bandpassHigh_ - bandpassLow_;

		// Quality factor
		double Q = centerFreq / bandwidth;

		// Normalized frequency
		double w0 = 2.0 * M_PI * centerFreq / sampleRate_;
		double alpha = std::sin(w0) / (2.0 * Q);

		// Bandpass filter coefficients
		double a0 = 1.0 + alpha;
		b0_ = alpha / a0;
		b1_ = 0.0;
		b2_ = -alpha / a0;
		a1_ = -2.0 * std::cos(w0) / a0;
		a2_ = (1.0 - alpha) / a0;

		// Initialize filter state
		x1_ = x2_ = y1_ = y2_ = 0.0;
	}

	float applyBandpassFilter(float input) {
		// Direct Form II implementation
		double output = b0_ * input + b1_ * x1_ + b2_ * x2_
					   - a1_ * y1_ - a2_ * y2_;

		// Update state
		x2_ = x1_;
		x1_ = input;
		y2_ = y1_;
		y1_ = output;

		return static_cast<float>(output);
	}

private:
	// Configuration
	int sampleRate_;
	int fftSize_;
	int numPeaks_;

	// PortAudio
	PaStream* stream_ = nullptr;
	std::atomic<bool> running_{false};

	// FFT
	fftw_plan plan_ = nullptr;
	double* fftIn_ = nullptr;
	fftw_complex* fftOut_ = nullptr;
	std::vector<double> window_;

	// Audio buffer
	std::vector<float> inputBuffer_;
	std::mutex bufferMutex_;
	std::atomic<bool> newDataAvailable_{false};

	// Results
	std::vector<FrequencyPeak> detectedPeaks_;
	std::mutex peaksMutex_;

	// Processing thread
	std::thread processingThread_;

	// Bandpass filter
	bool useBandpass_ = false;
	double bandpassLow_ = 0.0;
	double bandpassHigh_ = 0.0;

	// Biquad filter coefficients and state
	double b0_, b1_, b2_, a1_, a2_;
	double x1_, x2_, y1_, y2_;
};

// Demo program
int main(int argc, char** argv) {
	// Default settings matching rb3_node_cpp.cpp
	int sampleRate = 48000;
	int fftSize = 4096;
	int numPeaks = 10;
	double lowFreq = 4000.0;    // Bandpass low (matches rb3_node)
	double highFreq = 17000.0;  // Bandpass high (matches rb3_node)

	// Parse command line arguments
	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if (arg == "-sr" && i + 1 < argc) {
			sampleRate = std::atoi(argv[++i]);
		} else if (arg == "-fft" && i + 1 < argc) {
			fftSize = std::atoi(argv[++i]);
		} else if (arg == "-peaks" && i + 1 < argc) {
			numPeaks = std::atoi(argv[++i]);
		} else if (arg == "-low" && i + 1 < argc) {
			lowFreq = std::atof(argv[++i]);
		} else if (arg == "-high" && i + 1 < argc) {
			highFreq = std::atof(argv[++i]);
		} else if (arg == "-h" || arg == "--help") {
			std::cout << "Usage: " << argv[0] << " [options]\n"
					  << "Options:\n"
					  << "  -sr <rate>    Sample rate (default: 48000)\n"
					  << "  -fft <size>   FFT size (default: 4096)\n"
					  << "  -peaks <n>    Number of peaks to show (default: 5)\n"
					  << "  -low <freq>   Bandpass low cutoff in Hz (optional)\n"
					  << "  -high <freq>  Bandpass high cutoff in Hz (optional)\n"
					  << "  -h, --help    Show this help\n"
					  << "\nExamples:\n"
					  << "  " << argv[0] << " -fft 8192\n"
					  << "  " << argv[0] << " -low 800 -high 1200  (filter 800-1200 Hz)\n"
					  << "  " << argv[0] << " -low 400 -high 4000 -peaks 10\n";
			return 0;
		}
	}

	FrequencyDetector detector(sampleRate, fftSize, numPeaks, lowFreq, highFreq);

	if (!detector.start()) {
		std::cerr << "Failed to start detector.\n";
		return 1;
	}

	std::cout << "Press Enter to stop...\n";
	std::cin.get();

	detector.stop();
	return 0;
}
