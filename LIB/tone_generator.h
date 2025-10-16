#ifndef TONE_GENERATOR_H
#define TONE_GENERATOR_H

#include <vector>
#include <string>
#include <functional>

// Tone generator library - can be used standalone or embedded in other projects

class ToneGenerator {
public:
	struct Config {
		std::vector<double> frequencies;  // Frequencies to generate in Hz
		double sampleRate = 48000.0;      // Sample rate in Hz
		double duration = 5.0;            // Duration in seconds (0 = infinite)
		double gain = 0.8;                // Overall gain (0.0 - 1.0)
		int channels = 2;                 // 1 = mono, 2 = stereo
		double fadeTime = 0.01;           // Fade in/out time in seconds
	};

	ToneGenerator() = default;
	~ToneGenerator();

	// Start playing tones with given configuration
	// Returns true on success
	bool start(const Config& config);

	// Start playing asynchronously (non-blocking)
	// Optional callback when playback completes
	bool startAsync(const Config& config,
				   std::function<void()> onComplete = nullptr);

	// Stop playback
	void stop();

	// Check if currently playing
	bool isPlaying() const;

	// Get current playback position in seconds
	double getPlaybackPosition() const;

	// Wait for playback to complete (if duration was set)
	void waitForCompletion();

private:
	class Impl;
	Impl* impl_ = nullptr;
};

#endif // TONE_GENERATOR_H
