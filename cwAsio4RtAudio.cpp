/** @file       cwASIO4ALSA.cpp
 *  @brief      cwASIO driver that uses an ALSA device for audio I/O
 *  @author     Axel Holzinger, Stefan Heinzmann
 *  @version    1.0
 *  @date       2025
 *  @copyright  See file LICENSE in toplevel directory
 */

extern "C" {
    #include "cwASIOdriver.h"
}

#include <algorithm>
#include <atomic>
#include <bit>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <utility>

#include <RtAudio.h>

#define C4R_MAIN_DRIVER_KEY "cwAsio4RtAudio"
#define C4R_DRIVER_KEY_0    C4R_MAIN_DRIVER_KEY
#define C4R_DRIVER_KEY_1    C4R_MAIN_DRIVER_KEY " #1"
#define C4R_DRIVER_KEY_2    C4R_MAIN_DRIVER_KEY " #2"
#define C4R_DRIVER_KEY_3    C4R_MAIN_DRIVER_KEY " #3"
#define C4R_DRIVER_KEY_4    C4R_MAIN_DRIVER_KEY " #4"
#define C4R_DRIVER_KEY_5    C4R_MAIN_DRIVER_KEY " #5"
#define C4R_DRIVER_KEY_6    C4R_MAIN_DRIVER_KEY " #6"
#define C4R_DRIVER_KEY_7    C4R_MAIN_DRIVER_KEY " #7"
#define C4R_DRIVER_KEY_8    C4R_MAIN_DRIVER_KEY " #8"
#define C4R_DRIVER_KEY_9    C4R_MAIN_DRIVER_KEY " #9"

using namespace std::string_literals;


struct Configuration {
    std::pair<bool, std::string> outputDevice_;
    std::pair<bool, unsigned> outputChannels_;
    std::pair<bool, std::string> inputDevice_;
    std::pair<bool, unsigned> inputChannels_;
    unsigned sampleRate_;
    unsigned bufferSize_;

    Configuration()
        : outputDevice_{std::make_pair(false, std::string())}
        , outputChannels_{std::make_pair(false,unsigned(0))}
        , inputDevice_{std::make_pair(false, std::string())}
        , inputChannels_{std::make_pair(false,unsigned(0))}
        , sampleRate_{48000U}
        , bufferSize_{256U}
    {
    }
};

class CwAsio4AlsaDriver;
struct CallbackData {
    CwAsio4AlsaDriver *self_;
    bool useOutput_;
    bool useInput_;

    CallbackData() : self_{nullptr}, useOutput_{false}, useInput_{false} {}
    CallbackData(CwAsio4AlsaDriver *self, bool useOutput, bool useInput)
        : self_{self}
        , useOutput_{useOutput}
        , useInput_{useInput} {}
};


// Initialize the following data constants with the values for your driver.
char const *cwAsioKey               = "cwASIO";
size_t const cwAsioDriverKeyMaxLen  = 32U;
char const *cwAsioDriverDescription = "cwASIO for RtAudio devices";
long const  cwAsioDriverVersion     = 1;
size_t const errorBufLength         = 123U; // buffer is 124 chars
char const *configOutputDevice      = "outputDevice=";
char const *configOutputChannels    = "outputChannels=";
char const *configInputDevice       = "inputDevice=";
char const *configInputChannels     = "inputChannels=";
char const *configSampleRate        = "sampleRate=";
char const *configBufferSize        = "bufferSize=";

// we support 10 cwASIO devices of our kind
struct cwASIOinstance const cwAsioDriverInstances[] = {
    { .name = C4R_DRIVER_KEY_0, .guid = {0x754214be,0xf784,0x439e,0xa8,0x04,0x4e,0xf7,0x46,0x1f,0x26,0xc8} },
    { .name = C4R_DRIVER_KEY_1, .guid = {0x00af3471,0x800a,0x48bf,0xb6,0x74,0x4e,0xcc,0x84,0xcc,0x3a,0x0a} },
    { .name = C4R_DRIVER_KEY_2, .guid = {0x1b1a4acc,0x3518,0x4afa,0xad,0xee,0xc8,0x9b,0x3d,0xb1,0x13,0x06} },
    { .name = C4R_DRIVER_KEY_3, .guid = {0x1b82d5fb,0x3365,0x4ed3,0xa1,0x19,0x48,0x42,0x27,0xed,0xc3,0x54} },
    { .name = C4R_DRIVER_KEY_4, .guid = {0x7fd4c48d,0x002e,0x4294,0x90,0x53,0xab,0xf5,0x84,0x19,0xfc,0x01} },
    { .name = C4R_DRIVER_KEY_5, .guid = {0xf1d72885,0xf745,0x42db,0x99,0xaa,0x25,0x62,0xc0,0xf5,0x75,0xc0} },
    { .name = C4R_DRIVER_KEY_6, .guid = {0x4c27d025,0x0756,0x4097,0xb6,0xd4,0x5a,0xc6,0x39,0xdc,0xad,0xb9} },
    { .name = C4R_DRIVER_KEY_7, .guid = {0xee884f67,0xddd5,0x4587,0x8b,0xff,0xbf,0x26,0x8b,0xd6,0xdf,0xa5} },
    { .name = C4R_DRIVER_KEY_8, .guid = {0x5f75abba,0x7036,0x4d40,0xb9,0x4d,0xc0,0x68,0xa3,0x90,0x69,0xa0} },
    { .name = C4R_DRIVER_KEY_9, .guid = {0x1b10277c,0x64d8,0x4fa5,0xa7,0x32,0xc1,0xde,0x5c,0x64,0x7b,0x4a} },
    { .name = NULL }        // this terminates the list and must always be there.
};

std::atomic_uint activeInstances = 0;

static std::string to_string(RtAudioErrorType rtaError) {
    switch(rtaError) {
        default:
        case RTAUDIO_UNKNOWN_ERROR:     return "unspecified RtAudio error";
        case RTAUDIO_NO_ERROR:          return "";
        case RTAUDIO_WARNING:           return "non-critical RtAudio error";
        case RTAUDIO_NO_DEVICES_FOUND:  return "no RtAudio devices found on system";
        case RTAUDIO_INVALID_DEVICE:    return "an invalid RtAudio device ID was specified";
        case RTAUDIO_DEVICE_DISCONNECT: return "an RtAudio device in use was disconnected";
        case RTAUDIO_MEMORY_ERROR:      return "an error occurred during memory allocation";
        case RTAUDIO_INVALID_PARAMETER: return "an invalid parameter was specified to an RtAudio function";
        case RTAUDIO_INVALID_USE:       return "the RtAudio function was called incorrectly";
        case RTAUDIO_DRIVER_ERROR:      return "a system driver error occurred";
        case RTAUDIO_SYSTEM_ERROR:      return "a system error occurred";
        case RTAUDIO_THREAD_ERROR:      return "a thread error occurred";
    }
}

static cwASIOsampleType to_cwAsioSampleType(RtAudioFormat rtAudioFormats) {
    unsigned bits = 0;
    bool floatFormat = false;
    if(rtAudioFormats & RTAUDIO_FLOAT64) {
        bits = 64U;
        floatFormat = true;
    } else if(rtAudioFormats & RTAUDIO_FLOAT32) {
        bits = 32U;
        floatFormat = false;
    } else if(rtAudioFormats & RTAUDIO_SINT32) {
        bits = 32U;
    } else if(rtAudioFormats & RTAUDIO_SINT24) {
        bits = 24U;
    } else if(rtAudioFormats & RTAUDIO_SINT16) {
        bits = 16U;
    } else if(rtAudioFormats & RTAUDIO_SINT8) {
        bits = 8U;
    }
    if(floatFormat) {
        if(bits == 64U) {
            return std::endian::native == std::endian::little ? ASIOSTFloat64LSB : ASIOSTFloat64MSB;
        } else {
            return std::endian::native == std::endian::little ? ASIOSTFloat32LSB : ASIOSTFloat32MSB;
        }
    } else {
        switch(bits) {
            default:
            case  8U: return ASIOSTLastEntry;
            case 16U: return std::endian::native == std::endian::little ? ASIOSTInt16LSB : ASIOSTInt16MSB;
            case 24U: return std::endian::native == std::endian::little ? ASIOSTInt24LSB : ASIOSTInt24MSB;
            case 32U: return std::endian::native == std::endian::little ? ASIOSTInt32LSB : ASIOSTInt32MSB;
        }
    }
}

static size_t getSampleSizeInBytes(cwASIOsampleType ast) {
    switch(ast) {
        default:
            return 0;
        case ASIOSTInt16MSB:
        case ASIOSTInt16LSB:
            return 2U;
        case ASIOSTInt24MSB:
        case ASIOSTInt24LSB:
            return 3U;
        case ASIOSTInt32MSB:
        case ASIOSTInt32LSB:
        case ASIOSTInt32MSB16:
        case ASIOSTInt32LSB16:
        case ASIOSTInt32MSB18:
        case ASIOSTInt32LSB18:
        case ASIOSTInt32MSB20:
        case ASIOSTInt32LSB20:
        case ASIOSTInt32MSB24:
        case ASIOSTInt32LSB24:
        case ASIOSTFloat32MSB:
        case ASIOSTFloat32LSB:
            return 4U;
        case ASIOSTFloat64MSB:
        case ASIOSTFloat64LSB:
            return 8U;
    }
}

static long long getSystemTime() {
    struct timespec ts{ 0, 0 };
    [[maybe_unused]] int res = ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    assert(res >= 0);
    assert(ts.tv_sec >= 0);
    assert(ts.tv_nsec >= 0 && ts.tv_nsec < 1000000000L);
    return (long long) (ts.tv_sec * 1000000000ULL + ts.tv_nsec);
}

static std::pair<bool, std::string> getParameterString(std::string const &line, char const *parameterName) {
    size_t len = strlen(parameterName);
    if(line.length() >= len && line.substr(0, len) == parameterName)
        return std::make_pair(true, line.substr(len));
    return std::make_pair(false, std::string());
}

static std::pair<bool, unsigned> getParameterUnsigned(std::string const &line, char const *parameterName) {
    size_t len = strlen(parameterName);
    if(line.length() >= len && line.substr(0, len) == parameterName) {
        std::string parameter = line.substr(len);
        if(parameter.empty()) {
            return std::make_pair(true, std::numeric_limits<unsigned>::max());
        }
        try {
            unsigned long p = std::stoul(parameter);
            if (std::in_range<unsigned>(p)) {
                return std::make_pair(true, unsigned(p));
            }
        } catch(std::invalid_argument const&) {
        } catch(std::out_of_range const&) {
        } catch(...) {
        }
    }
    return std::make_pair(false, unsigned(0));
}

static std::pair<bool, Configuration> readConfiguration(std::filesystem::path const &pathToConfigFile) {
    Configuration config;
    try {
        if(!std::filesystem::exists(pathToConfigFile)) {
            return std::make_pair(false, Configuration());
        }
        std::ifstream file(pathToConfigFile.c_str());
        std::string line;
        while (std::getline(file, line)) {
            if(line.length() == 0 || std::isspace(line[0]) || line[0] == '#') {
                continue;
            }
            std::pair<bool, std::string> paramString = ::getParameterString(line, configOutputDevice);
            if(paramString.first) {
                config.outputDevice_ = paramString;
                continue;
            }
            std::pair<bool, unsigned> paramUnsigned = ::getParameterUnsigned(line, configOutputChannels);
            if(paramUnsigned.first) {
                config.outputChannels_ = paramUnsigned;
                continue;
            }
            paramString = ::getParameterString(line, configInputDevice);
            if(paramString.first) {
                config.inputDevice_ = paramString;
                continue;
            }
            paramUnsigned = ::getParameterUnsigned(line, configInputChannels);
            if(paramUnsigned.first) {
                config.inputChannels_ = paramUnsigned;
                continue;
            }
            paramUnsigned = ::getParameterUnsigned(line, configSampleRate);
            if(paramUnsigned.first) {
                config.sampleRate_ = paramUnsigned.second < std::numeric_limits<unsigned>::max() ? paramUnsigned.second : 48000U;
                continue;
            }
            paramUnsigned = ::getParameterUnsigned(line, configBufferSize);
            if(paramUnsigned.first) {
                config.bufferSize_ = paramUnsigned.second < std::numeric_limits<unsigned>::max() ? paramUnsigned.second : 256U;
                continue;
            }
        }
    } catch(const std::filesystem::filesystem_error&) {
        return std::make_pair(false, Configuration());
    } catch(...) {
        return std::make_pair(true, Configuration());
    }
    return std::make_pair(true, std::move(config));
}

static std::string getHomeDirectory() {
    char const *home = std::getenv("HOME");
    return std::string(home);
}

static void appendConfigPath(std::filesystem::path &path, std::string const &driverKey) {
    path /= driverKey;
    path /= "config";
}

static std::filesystem::path getUserConfigFilePath(std::string const &driverKey) {
    std::filesystem::path path = ::getHomeDirectory() / std::filesystem::path("."s + cwAsioKey);
    appendConfigPath(path, driverKey);
    return path;
}

static std::filesystem::path getSystemConfigFilePath(std::string const &driverKey) {
    std::filesystem::path path = "/etc" / std::filesystem::path(cwAsioKey);
    appendConfigPath(path, driverKey);
    return path;
}

static unsigned getDeviceId(RtAudio &rtAudio, std::string const &deviceName) {
    std::vector<std::string> names = rtAudio.getDeviceNames();
    auto cit = std::find(names.cbegin(), names.cend(), deviceName);
    if(cit == names.cend())
        return 0; // invalid device ID for device not found
    auto index = cit - names.cbegin();
    std::vector<unsigned> ids = rtAudio.getDeviceIds();
    assert(names.size() == ids.size());
    if(index >= ids.size()) {
        return 0; // invalid device ID for device not found
    }
    return ids.at(index);
}

static std::string makeStringWithMaxLength(char const *str, size_t maxLength) {
    assert(maxLength);
    if(maxLength == 0)
        return std::string();
    std::string s(str, str + maxLength);
    s[maxLength - 1U] = '\0';      // there might not be a null byte at the end of the string
    return std::string(s.c_str()); // return like this is needed to return shorter string
}

/** The cwASIO driver implemented as a C++ class. */
class CwAsio4AlsaDriver : public cwASIODriver {
    CwAsio4AlsaDriver(CwAsio4AlsaDriver &&) =delete;  // no move/copy

public:
    CwAsio4AlsaDriver()
        : cwASIODriver{ &vtbl_ }
        , references_{1}
        , instance_{nullptr}
        , initialised_{false}
        , deviceIdOut_{0}
        , outputChannels_{0}
        , deviceIdIn_{0}
        , inputChannels_{0}
        , cwAsioSampleType_{ASIOSTLastEntry}
        , sampleRate_{0}
        , bufferSize_{0}
        , preferBufferSwitchTimeInfo_{false}
        , pingNotPong_{true}
        , samplePos_{0}
    {
    }

    long queryInterface(cwASIOGUID const *guid, void **ptr) {
        *ptr = this;
        addRef();
        return 0;       // success
    }

    unsigned long addRef() {
        return references_.fetch_add(1) + 1;
    }

    unsigned long release() {
        unsigned long res = references_.fetch_sub(1) - 1;
        if (res == 0) {
            delete this;
            atomic_fetch_sub(&activeInstances, 1);
        }
        return res;
    }

    cwASIOBool init(void *sys) {
        if(initialised_)
            return ASIOFalse;
        if(!instance_) {
            errorText_ = "future with selector \"kcwASIOsetInstanceName\" not called";
            return ASIOFalse;
        if(rtaDeviceIds_.empty()) {
            errorText_ = "no RtAudio device available";
            return ASIOFalse;
        }
        // read stored configuration
        // first try user's home directory
        std::filesystem::path configFilePath = ::getUserConfigFilePath(instance_->name);
        std::pair<bool, Configuration> config = ::readConfiguration(configFilePath);
        if (!config.first) {
            configFilePath = ::getSystemConfigFilePath(instance_->name);
            config = ::readConfiguration(configFilePath);
        }
        unsigned deviceIdOut    = 0;
        unsigned outputChannels = 2U;
        unsigned deviceIdIn     = 0;
        unsigned inputChannels  = 2U;
        unsigned sampleRate     = 48000U;
        unsigned bufferSize     = 256U;
        if(config.first) {
            if(config.second.outputDevice_.first) {
                if(config.second.outputDevice_.second.empty())
                    deviceIdOut = rta_.getDefaultOutputDevice();
                else
                        deviceIdOut = getDeviceId(rta_, config.second.outputDevice_.second);
            }
            if(config.second.outputChannels_.first) {
                outputChannels = config.second.outputChannels_.second;
            }
            if(config.second.inputDevice_.first) {
                if(config.second.inputDevice_.second.empty())
                    deviceIdIn = rta_.getDefaultInputDevice();
                else
                        deviceIdIn = getDeviceId(rta_, config.second.inputDevice_.second);
            }
            if(config.second.inputChannels_.first) {
                inputChannels = config.second.inputChannels_.second;
            }
            sampleRate = config.second.sampleRate_;
            bufferSize = config.second.bufferSize_;
            if(!std::has_single_bit(bufferSize)) {
                bufferSize = std::bit_ceil(bufferSize);
            }
        }
        if(deviceIdOut == 0 && deviceIdIn == 0) {
            deviceIdOut = rta_.getDefaultOutputDevice();
            deviceIdIn  = rta_.getDefaultInputDevice();
        }
        cwASIOsampleType astOut = ::to_cwAsioSampleType(rta_.getDeviceInfo(deviceIdOut).nativeFormats);
        if(deviceIdOut && astOut == ASIOSTLastEntry) {
            errorText_ = "output device has unsupported sample format";
            return ASIOFalse;
        }
        cwASIOsampleType astIn = ::to_cwAsioSampleType(rta_.getDeviceInfo(deviceIdIn).nativeFormats);
        if(deviceIdIn && astIn == ASIOSTLastEntry) {
            errorText_ = "input device has unsupported sample format";
            return ASIOFalse;
        }
        if(deviceIdOut && deviceIdIn && astOut != astIn) {
            errorText_ = "output and input sample formats differ";
            return ASIOFalse;
        }
        RtAudio::StreamParameters paramsOut;
        RtAudio::StreamParameters *pParamsOut = nullptr;
        if(deviceIdOut) {
            RtAudio::DeviceInfo di = rta_.getDeviceInfo(deviceIdOut);
            if(di.outputChannels && outputChannels) {
                if(std::find(di.sampleRates.cbegin(), di.sampleRates.cend(), sampleRate) != di.sampleRates.cend()) {
                    paramsOut.deviceId = deviceIdOut;
                    paramsOut.nChannels = outputChannels == std::numeric_limits<unsigned>::max() ? di.outputChannels : outputChannels;
                    outputChannels = paramsOut.nChannels;
                    paramsOut.firstChannel = 0;
                    pParamsOut = &paramsOut;
                }
            }
        }
        RtAudio::StreamParameters paramsIn;
        RtAudio::StreamParameters *pParamsIn = nullptr;
        if(deviceIdIn) {
            RtAudio::DeviceInfo di = rta_.getDeviceInfo(deviceIdIn);
            if(di.inputChannels && inputChannels) {
                if(std::find(di.sampleRates.cbegin(), di.sampleRates.cend(), sampleRate) != di.sampleRates.cend()) {
                    paramsIn.deviceId = deviceIdIn;
                    paramsIn.nChannels = inputChannels == std::numeric_limits<unsigned>::max() ? di.inputChannels : inputChannels;
                    inputChannels = paramsIn.nChannels;
                    paramsIn.firstChannel = 0;
                    pParamsIn = &paramsIn;
                }
            }
        }
        if(!pParamsOut && !pParamsIn) {
            if(deviceIdOut || deviceIdIn)
                errorText_ = "device(s) doesn't/don't support chosen samplerate";
            else
                errorText_ = ::to_string(RTAUDIO_NO_DEVICES_FOUND) + "that have input and/or output channels";
            return ASIOFalse;
        }

        CallbackData cbd{this, pParamsOut ? true : false, pParamsIn ? true : false};
        rtaCbData_ = std::move(cbd);
        auto cb = [](void *outBuf, void *inBuf, unsigned frames, double streamTime, RtAudioStreamStatus status, void *cbd) {
            CallbackData *rtcbd = (CallbackData*) cbd;
            return rtcbd->self_->rtAudioCallback(rtcbd->useOutput_ ? outBuf : nullptr, rtcbd->useInput_ ? inBuf : nullptr, frames, streamTime, status);
        };
        RtAudioErrorType rtaError = rta_.openStream(pParamsOut, pParamsIn, RTAUDIO_SINT32, sampleRate, &bufferSize, cb, &rtaCbData_);
        if(rtaError != RTAUDIO_NO_ERROR) {
            errorText_ = "RtAudio::openStream: " + ::to_string(rtaError);
            return ASIOFalse;
        }

        deviceIdOut_      = deviceIdOut;
        outputChannels_   = outputChannels;
        deviceIdIn_       = deviceIdIn;
        inputChannels_    = inputChannels;
        cwAsioSampleType_ = astOut;
        sampleRate_       = sampleRate;
        bufferSize_       = bufferSize;
        initialised_      = true;
        return ASIOTrue;
    }

    void getDriverName(char *buf) {
        strcpy(buf, instance_->name);
    }

    long getDriverVersion() {
        return cwAsioDriverVersion;
    }

    void getErrorMessage(char *buf) {
        strncpy(buf, errorText_.c_str(), std::min(errorText_.length(), errorBufLength));
        buf[errorBufLength];
    }

    cwASIOError start() {
        if(!initialised_) {
            errorText_ = "not initialised";
            return ASE_InvalidMode;
        }
        RtAudioErrorType rtaError = rta_.startStream();
        if(rtaError != RTAUDIO_NO_ERROR) {
            errorText_ = "RtAudio::startStream: " + ::to_string(rtaError);
            return ASE_HWMalfunction;
        }
        return ASE_OK;
    }

    cwASIOError stop() {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(!rta_.isStreamOpen()) {
            errorText_ = "stream not open";
            return ASE_InvalidMode;
        }
        if(!rta_.isStreamRunning()) {
            errorText_ = "stream not started";
            return ASE_InvalidMode;
        }
        RtAudioErrorType rtaError = rta_.stopStream();
        if(rtaError != RTAUDIO_NO_ERROR) {
            errorText_ = "RtAudio::stopStream: " + ::to_string(rtaError);
            return ASE_HWMalfunction;
        }
        return ASE_OK;
    }

    cwASIOError getChannels(long *in, long *out) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(in)
            *in = long(inputChannels_);
        if(out)
            *out = long(outputChannels_);
        return ASE_OK;
    }

    cwASIOError getLatencies(long *in, long *out) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(in)
            *in = 0;
        if(out)
            *out = 0;
        return ASE_OK;
    }

    cwASIOError getBufferSize(long *min, long *max, long *pref, long *gran) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(min)
            *min = bufferSize_;
        if(max)
            *max = bufferSize_;
        if(pref)
            *pref = bufferSize_;
        if(gran)
            *gran = bufferSize_;
        return ASE_OK;
    }

    cwASIOError canSampleRate(double srate) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(srate < double(sampleRate_) || srate > double(sampleRate_)) {
            errorText_ = "samplerate not supported";
            return ASE_InvalidParameter;
        }
        return ASE_OK;
    }

    cwASIOError getSampleRate(double *srate) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(srate)
            *srate = sampleRate_;
        return ASE_OK;
    }

    cwASIOError setSampleRate(double srate) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(srate < double(sampleRate_) || srate > double(sampleRate_)) {
            errorText_ = "samplerate not supported";
            return ASE_InvalidParameter;
        }
        return ASE_OK;
    }

    cwASIOError getClockSources(struct cwASIOClockSource *clocks, long *num) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(clocks && num && *num > 0) {
            clocks->index = 0;
            clocks->associatedChannel = 0;
            clocks->associatedGroup = 0;
            clocks->isCurrentSource = ASIOTrue;
            strcpy(clocks->name, "internal");
            *num = 1;
        }
        return ASE_OK;
    }

    cwASIOError setClockSource(long ref) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(ref != 0) {
            errorText_ = "clock source not supported";
            return ASE_InvalidParameter;
        }
        return ASE_OK;
    }

    cwASIOError getSamplePosition(cwASIOSamples *sPos, cwASIOTimeStamp *tStamp) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(sPos)
            *sPos = 0;
        if(tStamp)
            *tStamp = 0;
        return ASE_OK;
    }

    cwASIOError getChannelInfo(struct cwASIOChannelInfo *info) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(info) {
            assert(info->isInput && deviceIdIn_ || !info->isInput && deviceIdOut_);
            RtAudio::DeviceInfo const &di = rta_.getDeviceInfo(info->isInput ? deviceIdIn_ : deviceIdOut_);
            unsigned maxChannels = info->isInput ? di.inputChannels : di.outputChannels;
            if(info->channel >= maxChannels) {
                errorText_ = "channel index too big";
                return ASE_InvalidParameter;
            }
            cwASIOsampleType ast = ::to_cwAsioSampleType(di.nativeFormats);
            if(ast == ASIOSTLastEntry) {
                errorText_ = "unsupported sample format";
                return ASE_InvalidParameter;
            }
            info->channelGroup = 0;
            info->isActive = ASIOTrue;
            info->type = ast;
            std::string name = "cwASIO "s + (info->isInput ? "input "s : "output "s) + "channel "s + std::to_string(info->channel + 1);
            strcpy(info->name, name.c_str());
        }
        return ASE_OK;
    }

    cwASIOError createBuffers(struct cwASIOBufferInfo *infos, long num, long size, struct cwASIOCallbacks const *cb) {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        if(size != 256) {
            errorText_ = "wrong size";
            return ASE_InvalidParameter;
        }
        if(num <= 0) {
            errorText_ = "num invalid";
            return ASE_InvalidParameter;
        }
        unsigned totalOutputChannels = 0;
        unsigned totalInputChannels = 0;
        for(unsigned n = 0; n < unsigned(num); n++) {
            infos->isInput ? totalInputChannels++ : totalOutputChannels++;
        }
        if(totalOutputChannels > getOutputChannels()) {
            errorText_ = "too many output channels";
            return ASE_InvalidParameter;
        }
        if(totalInputChannels > getInputChannels()) {
            errorText_ = "too many input channels";
            return ASE_InvalidParameter;
        }
        for(unsigned n = 0; n < unsigned(num); n++) {
            if(infos->isInput) {
                if (infos->channelNum >= getInputChannels()) {
                    errorText_ = "invalid input channelNum";
                    return ASE_InvalidParameter;
                }
            } else {
                if (infos->channelNum >= getOutputChannels()) {
                    errorText_ = "invalid output channelNum";
                    return ASE_InvalidParameter;
                }
            }
        }
        // allocate buffer memory for all potential channels (should be optimised later)
        size_t ssib = ::getSampleSizeInBytes(cwAsioSampleType_);
        for(unsigned n = 0; n < getOutputChannels(); n++) {
            std::vector<uint8_t> monoChannelBufA(size_t(size) * ssib, 0);
            std::vector<uint8_t> monoChannelBufB(size_t(size) * ssib, 0);
            std::pair<std::vector<uint8_t>, std::vector<uint8_t>> monoBufferPair = std::make_pair(std::move(monoChannelBufA), std::move(monoChannelBufB));
            cwAsioOutBufs_.push_back(std::move(monoBufferPair));
        }
        for(unsigned n = 0; n < getInputChannels(); n++) {
            std::vector<uint8_t> monoChannelBufA(size_t(size) * ssib, 0);
            std::vector<uint8_t> monoChannelBufB(size_t(size) * ssib, 0);
            std::pair<std::vector<uint8_t>, std::vector<uint8_t>> monoBufferPair = std::make_pair(std::move(monoChannelBufA), std::move(monoChannelBufB));
            cwAsioInBufs_.push_back(std::move(monoBufferPair));
        }
        for(unsigned n = 0; n < unsigned(num); n++) {
            if(infos[n].isInput) {
                infos[n].buffers[0] = cwAsioInBufs_.at(infos[n].channelNum).first.data();
                infos[n].buffers[1] = cwAsioInBufs_.at(infos[n].channelNum).second.data();
            } else {
                infos[n].buffers[0] = cwAsioOutBufs_.at(infos[n].channelNum).first.data();
                infos[n].buffers[1] = cwAsioOutBufs_.at(infos[n].channelNum).second.data();
            }
        }
        if (cb->asioMessage) {
            preferBufferSwitchTimeInfo_ = cb->asioMessage(kAsioSupportsTimeInfo, 0, nullptr, nullptr) == 1;
        }
        cwAsioCallbacks_ = *cb;
        return ASE_OK;
    }

    cwASIOError disposeBuffers() {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        cwAsioOutBufs_.clear();
        cwAsioInBufs_.clear();
        return ASE_OK;
    }

    cwASIOError controlPanel() {
        errorText_ = "not implemented";
        return ASE_NotPresent;
    }

    cwASIOError future(long sel, void *par) {
        switch(sel) {
            default:
                errorText_ = "future selector " + std::to_string(sel) + " not supported";
                return ASE_InvalidParameter;
            case kcwASIOsetInstanceName:
                if(!par) {
                    errorText_ = "future selector kcwASIOsetInstanceName didn't supply a driver name";
                    return ASE_InvalidParameter;
                }
                char const *parameter = (char const*) par;
                std::string driverKey = ::makeStringWithMaxLength(parameter, cwAsioDriverKeyMaxLen);
                for (struct cwASIOinstance const *entry = cwAsioDriverInstances; entry->name; ++entry) {
                    if(driverKey == entry->name) {
                        if(0 != cwASIOgetParameter(entry->name, NULL, NULL, 0))
                            break;      // not registered
                        if(instance_) {
                            if(instance_ != entry) {
                                std::cerr << "warning: different cwASIO drivers chosen in queryInterface (\""
                                          << instance_->name << "\") and in future (\"" << entry->name
                                          << "\") using the future one!\n";
                            }
                        }
                        instance_ = entry;
                        return ASE_SUCCESS;
                    }
                }
                return ASE_NotPresent;
        }
    }

    cwASIOError outputReady() {
        if(!initialised_) {
            errorText_ = std::string(instance_->name) + " not initialised";
            return ASE_InvalidMode;
        }
        errorText_ = "outputReady not supported";
        return ASE_NotPresent;
    }

private:
    int rtAudioCallback(void *outBuf, void *inBuf, unsigned frames, double time, RtAudioStreamStatus status) {
        size_t ssib = ::getSampleSizeInBytes(cwAsioSampleType_);
        // first copy data coming from the sound card to the cwASIO input buffers
        unsigned const inChans = getInputChannels();
        for(unsigned n = 0; n < inChans; n++) {
            uint8_t *cwAsioInBuf = pingNotPong_ ? cwAsioInBufs_[n].first.data() : cwAsioInBufs_[n].second.data();
            uint8_t *dst = cwAsioInBuf;
            uint8_t const *src = (uint8_t*) inBuf + n * ssib;
            for(unsigned f = 0; f < frames && f < 256U; f++) {
                assert(dst < cwAsioInBuf + frames * ssib);
                assert(src < (uint8_t*) inBuf + inChans * frames * ssib);
                ::memcpy(dst, src, ssib);
                dst += ssib;
                src += inChans * ssib;
            }
        }
        // now call the cwASIO callabck to hand over the data coming from the sound card and getting the data that will go to the sound card
        long const doubleBufferIndex = pingNotPong_ ? 0 : 1;
        bool copied = false;
        const int asioFlags = cwASIOTimeInfoFlags::kSystemTimeValid | cwASIOTimeInfoFlags::kSamplePositionValid
                            | cwASIOTimeInfoFlags::kSampleRateValid | cwASIOTimeInfoFlags::kSpeedValid;
        cwASIOTime cwAsioTime{0};
        cwAsioTime.timeInfo.speed = 1.0;
        cwAsioTime.timeInfo.systemTime = ::getSystemTime();
        cwAsioTime.timeInfo.samplePosition = samplePos_;
        cwAsioTime.timeInfo.sampleRate = 48000;
        cwAsioTime.timeInfo.flags = asioFlags;
        cwAsioTime.timeCode = cwASIOTimeCode{0};
        if(preferBufferSwitchTimeInfo_ && cwAsioCallbacks_.bufferSwitchTimeInfo) {
            cwAsioCallbacks_.bufferSwitchTimeInfo(&cwAsioTime, doubleBufferIndex, ASIOTrue);
            copied = true;
        } else if (cwAsioCallbacks_.bufferSwitch) {
            cwAsioCallbacks_.bufferSwitch(doubleBufferIndex, ASIOTrue);
            copied = true;
        }
        // last copy the data that comes from cwASIO to the sound card if we could call one of the callbacks, otherwise zero the data
        unsigned const outChans = getOutputChannels();
        for(unsigned n = 0; n < outChans; n++) {
            uint8_t const *cwAsioOutBuf = pingNotPong_ ? cwAsioOutBufs_[n].first.data() : cwAsioOutBufs_[n].second.data();
            uint8_t *dst = (uint8_t*) outBuf + n * ssib;;
            uint64_t zero = 0;
            uint8_t const *src = copied ? (uint8_t*) cwAsioOutBuf : (uint8_t*) &zero;
            for(unsigned f = 0; f < frames && f < 256U; f++) {
                assert(dst < (uint8_t*) outBuf + outChans * frames * ssib);
                if(copied)
                    assert(src < cwAsioOutBuf + frames * ssib);
                ::memcpy(dst, src, ssib);
                dst += outChans * ssib;
                if(copied)
                    src += ssib;
            }
        }
        samplePos_ += frames;
        pingNotPong_ = !pingNotPong_;
        return 0;
    }
    unsigned getOutputChannels() {
        if(deviceIdOut_ == 0)
            return 0;
        return rta_.getDeviceInfo(deviceIdOut_).outputChannels;
    }
    unsigned getInputChannels() {
        if(deviceIdIn_ == 0)
            return 0;
        return rta_.getDeviceInfo(deviceIdIn_).inputChannels;
    }


    static struct cwASIODriverVtbl const vtbl_;

    std::atomic_ulong references_;    // threadsafe reference counter
    cwASIOinstance const *instance_;  // which instance was selected
    bool initialised_;
    std::string errorText_;
    CallbackData rtaCbData_;
    RtAudio rta_;
    std::vector<unsigned> rtaDeviceIds_;
    unsigned deviceIdOut_;
    unsigned outputChannels_;
    unsigned deviceIdIn_;
    unsigned inputChannels_;
    cwASIOsampleType cwAsioSampleType_;
    unsigned sampleRate_;
    unsigned bufferSize_;
    bool preferBufferSwitchTimeInfo_;
    bool pingNotPong_;
    long long samplePos_;
    std::vector<std::pair<std::vector<uint8_t>, std::vector<uint8_t>>> cwAsioOutBufs_;
    std::vector<std::pair<std::vector<uint8_t>, std::vector<uint8_t>>> cwAsioInBufs_;
    cwASIOCallbacks cwAsioCallbacks_;
};

struct cwASIODriverVtbl const CwAsio4AlsaDriver::vtbl_ = {
    [](cwASIODriver *drv, cwASIOGUID const *guid, void **ptr){ return static_cast<CwAsio4AlsaDriver*>(drv)->queryInterface(guid, ptr); },
    [](cwASIODriver *drv){ return static_cast<CwAsio4AlsaDriver*>(drv)->addRef(); },
    [](cwASIODriver *drv){ return static_cast<CwAsio4AlsaDriver*>(drv)->release(); },
    [](cwASIODriver *drv, void *sys){ return static_cast<CwAsio4AlsaDriver*>(drv)->init(sys); },
    [](cwASIODriver *drv, char *buf){ static_cast<CwAsio4AlsaDriver*>(drv)->getDriverName(buf); },
    [](cwASIODriver *drv){ return static_cast<CwAsio4AlsaDriver*>(drv)->getDriverVersion(); },
    [](cwASIODriver *drv, char *buf){ return static_cast<CwAsio4AlsaDriver*>(drv)->getErrorMessage(buf); },
    [](cwASIODriver *drv){ return static_cast<CwAsio4AlsaDriver*>(drv)->start(); },
    [](cwASIODriver *drv){ return static_cast<CwAsio4AlsaDriver*>(drv)->stop(); },
    [](cwASIODriver *drv, long *in, long *out){ return static_cast<CwAsio4AlsaDriver*>(drv)->getChannels(in, out); },
    [](cwASIODriver *drv, long *in, long *out){ return static_cast<CwAsio4AlsaDriver*>(drv)->getLatencies(in, out); },
    [](cwASIODriver *drv, long *min, long *max, long *pref, long *gran){ return static_cast<CwAsio4AlsaDriver*>(drv)->getBufferSize(min, max, pref, gran); },
    [](cwASIODriver *drv, double srate){ return static_cast<CwAsio4AlsaDriver*>(drv)->canSampleRate(srate); },
    [](cwASIODriver *drv, double *srate){ return static_cast<CwAsio4AlsaDriver*>(drv)->getSampleRate(srate); },
    [](cwASIODriver *drv, double srate){ return static_cast<CwAsio4AlsaDriver*>(drv)->setSampleRate(srate); },
    [](cwASIODriver *drv, cwASIOClockSource *clocks, long *num){ return static_cast<CwAsio4AlsaDriver*>(drv)->getClockSources(clocks, num); },
    [](cwASIODriver *drv, long ref){ return static_cast<CwAsio4AlsaDriver*>(drv)->setClockSource(ref); },
    [](cwASIODriver *drv, cwASIOSamples *sPos, cwASIOTimeStamp *tStamp){ return static_cast<CwAsio4AlsaDriver*>(drv)->getSamplePosition(sPos, tStamp); },
    [](cwASIODriver *drv, cwASIOChannelInfo *info){ return static_cast<CwAsio4AlsaDriver*>(drv)->getChannelInfo(info); },
    [](cwASIODriver *drv, cwASIOBufferInfo *infos, long num, long size, cwASIOCallbacks const *cb){ return static_cast<CwAsio4AlsaDriver*>(drv)->createBuffers(infos, num, size, cb); },
    [](cwASIODriver *drv){ return static_cast<CwAsio4AlsaDriver*>(drv)->disposeBuffers(); },
    [](cwASIODriver *drv){ return static_cast<CwAsio4AlsaDriver*>(drv)->controlPanel(); },
    [](cwASIODriver *drv, long sel, void *par){ return static_cast<CwAsio4AlsaDriver*>(drv)->future(sel, par); },
    [](cwASIODriver *drv){ return static_cast<CwAsio4AlsaDriver*>(drv)->outputReady(); }
};

cwASIODriver *makeAsioDriver() {
    try {
        return new CwAsio4AlsaDriver();
    } catch(std::exception &ex) {
        return nullptr;
    }
}
