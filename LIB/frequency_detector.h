#ifndef FREQUENCY_DETECTOR_H
#define FREQUENCY_DETECTOR_H

#include <vector>
#include <functional>

// Frequency detector library - can be used standalone or embedded in other projects

class FrequencyDetector {
public:
	struct FrequencyPeak {
		double frequency;    // Hz
		double magnitude;    // normalized magnitude (0.0 - 1.0)
		double magnitudeDB;  // decibels
	};

	struct Config {
		int sampleRate = 48000;           // Sample rate in Hz
		int fftSize = 4096;               // FFT size (power of 2)
		int numPeaks = 5;                 // Number of peaks to detect
		double duration = 0.0;            // Duration in seconds (0 = infinite)
		double bandpassLow = 0.0;         // Low cutoff Hz (0 = disabled)
		double bandpassHigh = 0.0;        // High cutoff Hz (0 = disabled)
		double updateRate = 20.0;         // Updates per second
	};

	FrequencyDetector() = default;
	~FrequencyDetector();

	// Start detection with given configuration (blocking until duration expires or stop() called)
	// Returns detected frequencies
	std::vector<FrequencyPeak> start(const Config& config);

	// Start detection asynchronously (non-blocking)
	// Callback is called with detected peaks at specified update rate
	bool startAsync(const Config& config,
				   std::function<void(const std::vector<FrequencyPeak>&)> callback);

	// Stop detection
	void stop();

	// Check if currently detecting
	bool isDetecting() const;

	// Get most recent detected frequencies
	std::vector<FrequencyPeak> getDetectedFrequencies();

	// Wait for detection to complete (if duration was set)
	void waitForCompletion();

private:
	class Impl;
	Impl* impl_ = nullptr;
};

#endif // FREQUENCY_DETECTOR_H
