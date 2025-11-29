#include <portaudio.h>
#include <lame/lame.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <chrono>

class Mp3Recorder {
public:
Mp3Recorder() = default;
~Mp3Recorder() { stop(); }

// Start recording to 'path'. Returns true on success.
// sampleRate: 44100 or 48000 are common. channels: 1 (mono) or 2 (stereo).
// bitrateKbps: e.g., 128, 160, 192.
bool start(const std::string& path,
           int sampleRate = 48000,
           int channels = 1,
           int bitrateKbps = 128,
           unsigned long framesPerBuffer = 1024)
{
    if (running_.load()) {
        std::cerr << "Recorder already running.\n";
        return false;
    }

    // Basic argument checks
    if (channels != 1 && channels != 2) {
        std::cerr << "channels must be 1 or 2.\n";
        return false;
    }
    if (sampleRate <= 0) {
        std::cerr << "Invalid sample rate.\n";
        return false;
    }

    // Open output file
    fp_ = std::fopen(path.c_str(), "wb");
    if (!fp_) {
        std::perror("fopen");
        return false;
    }

    // Configure LAME
    lame_ = lame_init();
    if (!lame_) {
        std::cerr << "lame_init failed.\n";
        cleanupFile();
        return false;
    }
    lame_set_in_samplerate(lame_, sampleRate);
    lame_set_num_channels(lame_, channels);
    if (channels == 1) lame_set_mode(lame_, MONO);
    else               lame_set_mode(lame_, JOINT_STEREO);

    lame_set_brate(lame_, bitrateKbps);  // CBR bitrate
    lame_set_quality(lame_, 2);          // 2=high quality, slower; 5=faster

    if (lame_init_params(lame_) < 0) {
        std::cerr << "lame_init_params failed.\n";
        cleanupLame();
        cleanupFile();
        return false;
    }

    // Configure PortAudio input stream (blocking mode)
    PaError err;

    // NOTE: This class does NOT call Pa_Initialize/Pa_Terminate.
    // Ensure PortAudio is initialized before calling start().
    // If you want this class to manage PA lifetime, you can add Pa_Initialize here
    // and call Pa_Terminate in your program after stop().
    PaStreamParameters inParams;
    std::memset(&inParams, 0, sizeof(inParams));
    inParams.device = Pa_GetDefaultInputDevice();
    if (inParams.device == paNoDevice) {
        std::cerr << "No default input device.\n";
        cleanupLame();
        cleanupFile();
        return false;
    }
    inParams.channelCount = channels;
    inParams.sampleFormat = paInt16; // capture 16-bit PCM to match LAME encoder input easily
    inParams.suggestedLatency = Pa_GetDeviceInfo(inParams.device)->defaultLowInputLatency;
    inParams.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&stream_,
                        &inParams,
                        nullptr,
                        static_cast<double>(sampleRate),
                        framesPerBuffer,
                        paClipOff,
                        nullptr,        // no callback => blocking I/O
                        nullptr);
    if (err != paNoError) {
        std::cerr << "Pa_OpenStream: " << Pa_GetErrorText(err) << "\n";
        cleanupLame();
        cleanupFile();
        return false;
    }

    err = Pa_StartStream(stream_);
    if (err != paNoError) {
        std::cerr << "Pa_StartStream: " << Pa_GetErrorText(err) << "\n";
        cleanupStream();
        cleanupLame();
        cleanupFile();
        return false;
    }

    // Set state
    sampleRate_ = sampleRate;
    channels_ = channels;
    fpPath_ = path;
    framesPerBuffer_ = framesPerBuffer;
    running_.store(true);

    // Launch background thread
    worker_ = std::thread(&Mp3Recorder::threadFunc, this);
    return true;
}

// Stop recording and finalize file.
void stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return; // not running
    }
    if (worker_.joinable()) worker_.join();
    // Cleanup happens in thread after the loop and here as a safety net
    cleanupStream();
    flushAndCloseEncoder();
    cleanupFile();
}

bool isRecording() const {
    return running_.load();
}

private:
void cleanupLame() {
if (lame_) {
lame_close(lame_);
lame_ = nullptr;
}
}
void threadFunc() {
// Buffers
std::vector<int16_t> pcm(framesPerBuffer_ * channels_);
// mp3 buffer size: per LAME docs, at least 1.25 * PCM + 7200 bytes
const int mp3BufSize = static_cast<int>(1.25 * pcm.size() + 7200);
std::vector<unsigned char> mp3buf(mp3BufSize);

    while (running_.load()) {
        PaError err = Pa_ReadStream(stream_, pcm.data(), framesPerBuffer_);
        if (err == paInputOverflowed) {
            // Overflow: continue, but data may have gaps
            std::cerr << "Warning: input overflow.\n";
            continue;
        } else if (err != paNoError) {
            std::cerr << "Pa_ReadStream: " << Pa_GetErrorText(err) << "\n";
            break;
        }

        int bytes = 0;
        if (channels_ == 2) {
            // Interleaved short PCM for stereo
            bytes = lame_encode_buffer_interleaved(lame_,
                                                   pcm.data(),
                                                   static_cast<int>(framesPerBuffer_),
                                                   mp3buf.data(),
                                                   mp3BufSize);
        } else {
            // Mono: provide left channel only
            bytes = lame_encode_buffer(lame_,
                                       pcm.data(),   // left
                                       nullptr,      // right
                                       static_cast<int>(framesPerBuffer_),
                                       mp3buf.data(),
                                       mp3BufSize);
        }

        if (bytes < 0) {
            std::cerr << "LAME encode error: " << bytes << "\n";
            break;
        }

        if (bytes > 0) {
            size_t written = std::fwrite(mp3buf.data(), 1, static_cast<size_t>(bytes), fp_);
            if (written != static_cast<size_t>(bytes)) {
                std::perror("fwrite");
                break;
            }
        }
    }

    // Flush remaining MP3 frames
    flushAndCloseEncoder();
    // Stop and close stream
    cleanupStream();
    // Close file
    cleanupFile();
    running_.store(false);
}

void flushAndCloseEncoder() {
    if (lame_) {
        unsigned char flushBuf[7200];
        int bytes = lame_encode_flush(lame_, flushBuf, sizeof(flushBuf));
        if (bytes > 0 && fp_) {
            (void)std::fwrite(flushBuf, 1, static_cast<size_t>(bytes), fp_);
        }
        lame_close(lame_);
        lame_ = nullptr;
    }
}

void cleanupStream() {
    if (stream_) {
        Pa_StopStream(stream_);
        Pa_CloseStream(stream_);
        stream_ = nullptr;
    }
}

void cleanupFile() {
    if (fp_) {
        std::fclose(fp_);
        fp_ = nullptr;
    }
}

private:
// Runtime
std::atomic<bool> running_{false};
std::thread worker_;

// Resources
PaStream* stream_ = nullptr;
lame_t lame_ = nullptr;
FILE* fp_ = nullptr;

// Config
std::string fpPath_;
int sampleRate_ = 0;
int channels_ = 0;
unsigned long framesPerBuffer_ = 1024;

};

// Minimal demo: record 5 seconds of mono 48 kHz at 128 kbps to out.mp3
int main() {
PaError err = Pa_Initialize();
if (err != paNoError) {
std::cerr << "Pa_Initialize: " << Pa_GetErrorText(err) << "\n";
return 1;
}

Mp3Recorder rec;
if (!rec.start("out.mp3", 48000, 1, 128)) {
    std::cerr << "Failed to start recording.\n";
    Pa_Terminate();
    return 1;
}

std::cout << "Recording for 5 seconds...\n";
std::this_thread::sleep_for(std::chrono::seconds(5));
rec.stop();
std::cout << "Saved to out.mp3\n";

Pa_Terminate();
return 0;

}