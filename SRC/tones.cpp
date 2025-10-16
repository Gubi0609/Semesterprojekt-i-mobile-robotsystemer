#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <portaudio.h>
// THIS IS ENETIRELY GENERATED CODE BY CHATGPT I HAVE NOT LOOKED AT IT AND I DONT KNOW WHAT IT DOES OTHER THAT IT PLAYS TONES
// Simple command-line tool to play one or more sine wave tones using PortAudio
// Example usage: ./tones 5000 10000 -d 5 -sr 48000 -gain 0.6 -ch 2 (first 2 args are frequencies in Hz -d is duration in seconds, -sr is sample rate, -gain is overall gain, -ch is number of channels (1=mono, 2=stereo))
// Compile with: g++ -o tones tones.cpp -lportaudio -lm
// Requires PortAudio library: http://www.portaudio.com/
struct SynthData {
std::vector<double> freq;
std::vector<double> phase;
double sampleRate = 48000.0;
double amp = 0.8;           // overall gain (after normalization)
long long totalFrames = 0;  // duration in frames
long long framesDone = 0;
long long rampFrames = 0;   // fade-in/out length in frames
int numChannels = 2;        // stereo
};

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

const size_t N = data->freq.size();
const double sr = data->sampleRate;
const double norm = (N > 0) ? (1.0 / static_cast<double>(N)) : 0.0;

for (unsigned long i = 0; i < frameCount; ++i) {
    if (data->framesDone >= data->totalFrames) {
        // Fill silence once complete
        for (int ch = 0; ch < data->numChannels; ++ch) *out++ = 0.0f;
        continue;
    }

    double sample = 0.0;
    for (size_t k = 0; k < N; ++k) {
        sample += std::sin(data->phase[k]);
        // advance phase
        double inc = (2.0 * M_PI * data->freq[k]) / sr;
        data->phase[k] += inc;
        if (data->phase[k] >= 2.0 * M_PI) data->phase[k] -= 2.0 * M_PI;
    }
    sample *= norm;

    // Fade-in/out to avoid clicks
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
        *out++ = outSample; // same signal to both L/R (mono-to-stereo)
    }

    data->framesDone++;
}

if (data->framesDone >= data->totalFrames) return paComplete;
return paContinue;

}

int main(int argc, char** argv) {
// Defaults
double sampleRate = 48000.0;
double durationSec = 5.0;
int channels = 2;
double gain = 0.8; // overall gain; with multiple tones, actual level is gain/N
std::vector<double> freqs;

// Simple CLI parsing:
// Usage examples:
//   ./tones 5000 10000 -d 5 -sr 48000 -gain 0.6 -ch 2
for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-d" && i + 1 < argc) {
        durationSec = std::atof(argv[++i]);
    } else if (a == "-sr" && i + 1 < argc) {
        sampleRate = std::atof(argv[++i]);
    } else if (a == "-gain" && i + 1 < argc) {
        gain = std::atof(argv[++i]);
    } else if (a == "-ch" && i + 1 < argc) {
        channels = std::atoi(argv[++i]);
        channels = std::max(1, std::min(channels, 2)); // mono or stereo
    } else {
        // treat as frequency (Hz)
        double f = std::atof(a.c_str());
        if (f > 0) freqs.push_back(f);
    }
}

if (freqs.empty()) {
    // default frequencies if none provided
    freqs = {1000.0};
}

// Nyquist check
double maxF = 0.0;
for (double f : freqs) maxF = std::max(maxF, f);
if (sampleRate < 2.0 * maxF) {
    std::cerr << "Warning: sampleRate < 2x max frequency; increase -sr to avoid aliasing.\n";
}

SynthData data;
data.freq = freqs;
data.phase.assign(freqs.size(), 0.0);
data.sampleRate = sampleRate;
data.amp = gain;
data.numChannels = channels;
data.totalFrames = static_cast<long long>(durationSec * sampleRate);
data.framesDone = 0;
data.rampFrames = std::max(1LL, static_cast<long long>(0.01 * sampleRate)); // 10 ms ramp

PaError err = Pa_Initialize();
if (err != paNoError) {
    std::cerr << "Pa_Initialize error: " << Pa_GetErrorText(err) << "\n";
    return 1;
}

PaStreamParameters outParams;
outParams.device = Pa_GetDefaultOutputDevice();
if (outParams.device == paNoDevice) {
    std::cerr << "No default output device.\n";
    Pa_Terminate();
    return 1;
}
outParams.channelCount = data.numChannels;
outParams.sampleFormat = paFloat32;
outParams.suggestedLatency = Pa_GetDeviceInfo(outParams.device)->defaultLowOutputLatency;
outParams.hostApiSpecificStreamInfo = nullptr;

PaStream* stream = nullptr;
err = Pa_OpenStream(&stream,
                    nullptr,
                    &outParams,
                    data.sampleRate,
                    256,                // frames per buffer
                    paClipOff,
                    paCallback,
                    &data);
if (err != paNoError) {
    std::cerr << "Pa_OpenStream error: " << Pa_GetErrorText(err) << "\n";
    Pa_Terminate();
    return 1;
}

err = Pa_StartStream(stream);
if (err != paNoError) {
    std::cerr << "Pa_StartStream error: " << Pa_GetErrorText(err) << "\n";
    Pa_CloseStream(stream);
    Pa_Terminate();
    return 1;
}

// Wait until stream finishes (callback returns paComplete)
while (Pa_IsStreamActive(stream) == 1) {
    Pa_Sleep(50);
}

Pa_CloseStream(stream);
Pa_Terminate();
return 0;

}