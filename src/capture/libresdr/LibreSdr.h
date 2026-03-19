/// @file LibreSdr.h
/// @class LibreSdr
/// @brief A class to capture data on the LibreSDR (dual RX) via SoapySDR.
/// @details Uses the SoapySDR C++ API to stream from both RX channels
/// simultaneously. RX0 is mapped to the reference channel and RX1 to
/// the surveillance channel. The dual TX channels are not used.

#ifndef LIBRESDR_H
#define LIBRESDR_H

#include "capture/Source.h"
#include "data/IqData.h"

#include <cstdint>
#include <string>
#include <vector>

// Forward-declare SoapySDR types to avoid exposing the header here.
namespace SoapySDR { class Device; class Stream; }

class LibreSdr : public Source
{
private:
  /// @brief Per-channel RX gain (dB). Index 0 = reference, 1 = surveillance.
  std::vector<double> gain;

  /// @brief Per-channel antenna port name (e.g. "LNAW").
  std::vector<std::string> antenna;

  /// @brief Analog filter bandwidth (Hz). 0 = automatic.
  double bandwidth;

  /// @brief SoapySDR driver/device argument string (e.g. "driver=lime").
  std::string deviceArgs;

  /// @brief Pointer to the opened SoapySDR device.
  SoapySDR::Device *soapyDevice;

  /// @brief Pointer to the active RX stream.
  SoapySDR::Stream *rxStream;

public:
  /// @brief Constructor.
  /// @param type Device type string ("LibreSDR").
  /// @param fc Center frequency (Hz).
  /// @param fs Sampling frequency (Hz).
  /// @param path Absolute path to IQ save location.
  /// @param saveIq Pointer to IQ save flag.
  /// @param gain Per-channel gain vector.
  /// @param antenna Per-channel antenna port vector.
  /// @param bandwidth Analog filter bandwidth (Hz), 0 for auto.
  /// @param deviceArgs SoapySDR device argument string.
  LibreSdr(std::string type, uint32_t fc, uint32_t fs,
           std::string path, bool *saveIq,
           std::vector<double> gain,
           std::vector<std::string> antenna,
           double bandwidth,
           std::string deviceArgs);

  /// @brief Destructor — cleans up SoapySDR resources.
  ~LibreSdr();

  /// @brief Open and configure the SoapySDR device.
  void start() override;

  /// @brief Deactivate and close the RX stream and device.
  void stop() override;

  /// @brief Continuously read dual-channel RX samples into the buffers.
  /// @param buffer1 Reference channel buffer (RX0).
  /// @param buffer2 Surveillance channel buffer (RX1).
  void process(IqData *buffer1, IqData *buffer2) override;

  /// @brief Replay IQ data from a file.
  /// @param buffer1 Reference channel buffer.
  /// @param buffer2 Surveillance channel buffer.
  /// @param file Path to the replay file.
  /// @param loop Whether to loop on EOF.
  void replay(IqData *buffer1, IqData *buffer2,
              std::string file, bool loop) override;
};

#endif // LIBRESDR_H