/// @file LibreSdr.h
/// @class LibreSdr
/// @brief A class to capture data on the LibreSDR (dual RX AD9363) via libiio.
/// @details Uses libiio to stream from both RX channels simultaneously.
/// RX0 is mapped to the reference channel and RX1 to the surveillance channel.
/// The dual TX channels are not used.
/// Works over both USB and network (IP) connections.

#ifndef LIBRESDR_H
#define LIBRESDR_H

#include "capture/Source.h"
#include "data/IqData.h"

#include <cstdint>
#include <string>
#include <vector>

// Forward-declare libiio types
struct iio_context;
struct iio_device;
struct iio_channel;
struct iio_buffer;

class LibreSdr : public Source
{
private:
  /// @brief Per-channel RX gain (dB). Index 0 = reference, 1 = surveillance.
  std::vector<double> gain;

  /// @brief Analog filter bandwidth (Hz). 0 = automatic.
  double bandwidth;

  /// @brief IIO URI for device connection (e.g. "usb:" or "ip:192.168.2.1").
  std::string uri;

  /// @brief Number of samples per buffer read.
  size_t bufferSize;

  /// @brief libiio context.
  iio_context *ctx;

  /// @brief AD9363 PHY device (for configuration).
  iio_device *phyDev;

  /// @brief RX streaming device (e.g. cf-ad9361-lpc).
  iio_device *rxDev;

  /// @brief RX channel I/Q pointers for channel 0 and 1.
  iio_channel *rx0_i;
  iio_channel *rx0_q;
  iio_channel *rx1_i;
  iio_channel *rx1_q;

  /// @brief libiio RX buffer.
  iio_buffer *rxBuf;

  /// @brief Helper to configure an AD9363 RX channel via PHY attributes.
  /// @param phyChan PHY channel index (0 or 1).
  /// @param gainDb Gain in dB.
  void configure_rx_channel(int phyChan, double gainDb);

public:
  /// @brief Constructor.
  /// @param type Device type string ("LibreSDR").
  /// @param fc Center frequency (Hz).
  /// @param fs Sampling frequency (Hz).
  /// @param path Absolute path to IQ save location.
  /// @param saveIq Pointer to IQ save flag.
  /// @param gain Per-channel gain vector.
  /// @param bandwidth Analog filter bandwidth (Hz), 0 for auto.
  /// @param uri IIO URI string (e.g. "usb:" or "ip:192.168.2.1").
  /// @param bufferSize Number of samples per buffer read.
  LibreSdr(std::string type, uint32_t fc, uint32_t fs,
           std::string path, bool *saveIq,
           std::vector<double> gain,
           double bandwidth,
           std::string uri,
           size_t bufferSize);

  /// @brief Destructor — cleans up libiio resources.
  ~LibreSdr();

  /// @brief Open and configure the libiio device with dual RX.
  void start() override;

  /// @brief Destroy the IIO buffer and context.
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