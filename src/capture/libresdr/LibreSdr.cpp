#include "LibreSdr.h"

#include <iostream>
#include <complex>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <thread>
#include <chrono>
#include <stdexcept>

#include <iio.h>

// constructor
LibreSdr::LibreSdr(std::string _type, uint32_t _fc, uint32_t _fs,
                   std::string _path, bool *_saveIq,
                   std::vector<double> _gain,
                   double _bandwidth,
                   std::string _uri,
                   size_t _bufferSize)
    : Source(_type, _fc, _fs, _path, _saveIq),
      gain(std::move(_gain)),
      bandwidth(_bandwidth),
      uri(std::move(_uri)),
      bufferSize(_bufferSize),
      ctx(nullptr),
      phyDev(nullptr),
      rxDev(nullptr),
      rx0_i(nullptr), rx0_q(nullptr),
      rx1_i(nullptr), rx1_q(nullptr),
      rxBuf(nullptr)
{
}

LibreSdr::~LibreSdr()
{
  stop();
}

void LibreSdr::configure_rx_channel(int phyChan, double gainDb)
{
  // PHY channel names: "voltage0" for RX1, "voltage1" for RX2
  std::string chanName = "voltage" + std::to_string(phyChan);
  struct iio_channel *phyCh = iio_device_find_channel(phyDev, chanName.c_str(), false);
  if (!phyCh)
  {
    std::cerr << "Error: LibreSDR - cannot find PHY channel " << chanName << std::endl;
    return;
  }

  // Set gain mode to manual
  iio_channel_attr_write(phyCh, "gain_control_mode", "manual");

  // Set hardware gain
  iio_channel_attr_write_double(phyCh, "hardwaregain", gainDb);

  // Set RF bandwidth if specified
  if (bandwidth > 0)
  {
    iio_channel_attr_write_longlong(phyCh, "rf_bandwidth", static_cast<long long>(bandwidth));
  }

  // Set sampling frequency (on channel 0 it applies to both)
  if (phyChan == 0)
  {
    iio_channel_attr_write_longlong(phyCh, "sampling_frequency", static_cast<long long>(fs));
  }

  std::cout << "LibreSDR: Configured PHY " << chanName
            << " gain=" << gainDb << " dB" << std::endl;
}

void LibreSdr::start()
{
  std::cout << "LibreSDR: Connecting via URI: \"" << uri << "\"" << std::endl;

  // Create IIO context
  if (uri.empty() || uri == "usb:" || uri == "usb")
  {
    // Scan for USB devices
    ctx = iio_create_default_context();
    if (!ctx)
    {
      ctx = iio_create_context_from_uri("usb:");
    }
  }
  else
  {
    ctx = iio_create_context_from_uri(uri.c_str());
  }

  if (!ctx)
  {
    std::cerr << "Error: LibreSDR - Failed to create IIO context for URI: \""
              << uri << "\"" << std::endl;
    std::cerr << "  For USB: uri: \"usb:\"" << std::endl;
    std::cerr << "  For network: uri: \"ip:192.168.2.1\"" << std::endl;
    return;
  }

  unsigned int devCount = iio_context_get_devices_count(ctx);
  std::cout << "LibreSDR: IIO context has " << devCount << " device(s)." << std::endl;
  for (unsigned int i = 0; i < devCount; i++)
  {
    const struct iio_device *dev = iio_context_get_device(ctx, i);
    const char *name = iio_device_get_name(dev);
    std::cout << "  Device " << i << ": " << (name ? name : "(unnamed)") << std::endl;
  }

  // Find the AD9363/AD9361 PHY device
  phyDev = iio_context_find_device(ctx, "ad9361-phy");
  if (!phyDev)
  {
    std::cerr << "Error: LibreSDR - Cannot find ad9361-phy device." << std::endl;
    iio_context_destroy(ctx);
    ctx = nullptr;
    return;
  }

  // Enable dual RX channel mode via PHY attribute if available
  // The LibreSDR with AD9363 needs to be in 2R2T mode
  struct iio_channel *phyCh0 = iio_device_find_channel(phyDev, "voltage0", false);
  if (phyCh0)
  {
    // Check if there's 2 RX channels available in the streaming device
    // First set the RX LO frequency (applies to both channels)
    struct iio_channel *rxLo = iio_device_find_channel(phyDev, "altvoltage0", true);
    if (rxLo)
    {
      iio_channel_attr_write_longlong(rxLo, "frequency", static_cast<long long>(fc));
      std::cout << "LibreSDR: Set RX LO frequency to " << fc << " Hz" << std::endl;
    }
    else
    {
      std::cerr << "Warning: LibreSDR - Cannot find RX LO channel." << std::endl;
    }
  }

  // Configure PHY channels
  if (gain.size() > 0)
  {
    configure_rx_channel(0, gain[0]);
  }
  if (gain.size() > 1)
  {
    configure_rx_channel(1, gain[1]);
  }

  // Find the RX streaming device
  rxDev = iio_context_find_device(ctx, "cf-ad9361-lpc");
  if (!rxDev)
  {
    std::cerr << "Error: LibreSDR - Cannot find cf-ad9361-lpc streaming device." << std::endl;
    std::cerr << "  The LibreSDR firmware may need to be configured for 2R2T mode." << std::endl;
    iio_context_destroy(ctx);
    ctx = nullptr;
    return;
  }

  // Find RX streaming channels
  // Channel naming: voltage0 (RX1 I), voltage1 (RX1 Q), voltage2 (RX2 I), voltage3 (RX2 Q)
  rx0_i = iio_device_find_channel(rxDev, "voltage0", false);
  rx0_q = iio_device_find_channel(rxDev, "voltage1", false);
  rx1_i = iio_device_find_channel(rxDev, "voltage2", false);
  rx1_q = iio_device_find_channel(rxDev, "voltage3", false);

  if (!rx0_i || !rx0_q)
  {
    std::cerr << "Error: LibreSDR - Cannot find RX0 I/Q channels (voltage0/voltage1)." << std::endl;
    iio_context_destroy(ctx);
    ctx = nullptr;
    return;
  }

  if (!rx1_i || !rx1_q)
  {
    std::cerr << "Error: LibreSDR - Cannot find RX1 I/Q channels (voltage2/voltage3)." << std::endl;
    std::cerr << "  The LibreSDR firmware must be in 2R2T mode for dual RX." << std::endl;
    std::cerr << "  Check: ssh root@<pluto_ip> then: fw_printenv compatible" << std::endl;
    std::cerr << "  Should contain 'ad9361' not 'ad9364' for 2-channel mode." << std::endl;
    iio_context_destroy(ctx);
    ctx = nullptr;
    return;
  }

  // Enable all 4 channels for streaming
  iio_channel_enable(rx0_i);
  iio_channel_enable(rx0_q);
  iio_channel_enable(rx1_i);
  iio_channel_enable(rx1_q);

  std::cout << "LibreSDR: Enabled 4 RX streaming channels (2x I/Q)." << std::endl;

  // Create the RX buffer
  rxBuf = iio_device_create_buffer(rxDev, bufferSize, false);
  if (!rxBuf)
  {
    std::cerr << "Error: LibreSDR - Failed to create RX buffer (size="
              << bufferSize << ")." << std::endl;
    iio_context_destroy(ctx);
    ctx = nullptr;
    return;
  }

  std::cout << "LibreSDR: Created RX buffer with " << bufferSize << " samples." << std::endl;
  std::cout << "LibreSDR: Capture started successfully." << std::endl;
}

void LibreSdr::stop()
{
  if (rxBuf)
  {
    iio_buffer_destroy(rxBuf);
    rxBuf = nullptr;
  }
  if (ctx)
  {
    iio_context_destroy(ctx);
    ctx = nullptr;
  }
  phyDev = nullptr;
  rxDev = nullptr;
  rx0_i = rx0_q = rx1_i = rx1_q = nullptr;
}

void LibreSdr::process(IqData *buffer1, IqData *buffer2)
{
  if (!ctx || !rxBuf)
  {
    std::cerr << "Error: LibreSDR - device not started." << std::endl;
    return;
  }

  // Get the step size (bytes between consecutive samples of the same channel)
  const ptrdiff_t sampleSize = static_cast<ptrdiff_t>(iio_buffer_step(rxBuf));

  while (true)
  {
    // Refill the buffer (blocking read)
    ssize_t nbytes = iio_buffer_refill(rxBuf);
    if (nbytes < 0)
    {
      std::cerr << "Error: LibreSDR - iio_buffer_refill returned "
                << nbytes << std::endl;
      break;
    }

    // Get pointers to channel data
    const char *rx0_i_start = static_cast<const char *>(iio_buffer_first(rxBuf, rx0_i));
    const char *rx0_q_start = static_cast<const char *>(iio_buffer_first(rxBuf, rx0_q));
    const char *rx1_i_start = static_cast<const char *>(iio_buffer_first(rxBuf, rx1_i));
    const char *rx1_q_start = static_cast<const char *>(iio_buffer_first(rxBuf, rx1_q));
    const char *bufEnd = static_cast<const char *>(iio_buffer_end(rxBuf));

    // AD9361 samples are 16-bit signed integers (SC16)
    size_t nSamples = 0;

    buffer1->lock();
    buffer2->lock();

    const char *p0i = rx0_i_start;
    const char *p0q = rx0_q_start;
    const char *p1i = rx1_i_start;
    const char *p1q = rx1_q_start;

    while (p0i < bufEnd && p1i < bufEnd)
    {
      // Read 16-bit signed samples and convert to double
      int16_t i0 = *reinterpret_cast<const int16_t *>(p0i);
      int16_t q0 = *reinterpret_cast<const int16_t *>(p0q);
      int16_t i1 = *reinterpret_cast<const int16_t *>(p1i);
      int16_t q1 = *reinterpret_cast<const int16_t *>(p1q);

      buffer1->push_back({static_cast<double>(i0), static_cast<double>(q0)});
      buffer2->push_back({static_cast<double>(i1), static_cast<double>(q1)});

      p0i += sampleSize;
      p0q += sampleSize;
      p1i += sampleSize;
      p1q += sampleSize;
      nSamples++;
    }

    buffer1->unlock();
    buffer2->unlock();

    // Save IQ data to file if enabled
    if (*saveIq && nSamples > 0)
    {
      // Save as interleaved SC16: [I0 Q0] [I0 Q0] ... [I1 Q1] [I1 Q1] ...
      // Rewrite from buffer pointers
      saveIqFile.write(rx0_i_start, nbytes / 2);
      saveIqFile.write(rx1_i_start, nbytes / 2);
    }
  }
}

void LibreSdr::replay(IqData *buffer1, IqData *buffer2,
                      std::string file, bool loop)
{
  std::ifstream inFile(file, std::ios::binary);
  if (!inFile.is_open())
  {
    std::cerr << "Error: LibreSDR - cannot open replay file: " << file << std::endl;
    return;
  }

  // Replay file format: SC16 interleaved, channel 0 block then channel 1 block
  const size_t blockSamples = 1024;
  // Each sample is 2 x int16_t (I and Q)
  const size_t sampleBytes = 2 * sizeof(int16_t);
  std::vector<int16_t> rawBuf0(blockSamples * 2);
  std::vector<int16_t> rawBuf1(blockSamples * 2);

  while (true)
  {
    // Read channel 0 block
    inFile.read(reinterpret_cast<char *>(rawBuf0.data()),
                blockSamples * sampleBytes);
    std::streamsize bytesRead0 = inFile.gcount();
    size_t samplesRead0 = bytesRead0 / sampleBytes;

    // Read channel 1 block
    inFile.read(reinterpret_cast<char *>(rawBuf1.data()),
                blockSamples * sampleBytes);
    std::streamsize bytesRead1 = inFile.gcount();
    size_t samplesRead1 = bytesRead1 / sampleBytes;

    size_t nSamples = std::min(samplesRead0, samplesRead1);

    if (nSamples == 0)
    {
      if (loop)
      {
        inFile.clear();
        inFile.seekg(0, std::ios::beg);
        continue;
      }
      break;
    }

    buffer1->lock();
    buffer2->lock();
    for (size_t i = 0; i < nSamples; i++)
    {
      buffer1->push_back({static_cast<double>(rawBuf0[i * 2]),
                          static_cast<double>(rawBuf0[i * 2 + 1])});
      buffer2->push_back({static_cast<double>(rawBuf1[i * 2]),
                          static_cast<double>(rawBuf1[i * 2 + 1])});
    }
    buffer1->unlock();
    buffer2->unlock();

    // Throttle replay to approximate real-time
    double durationSec = static_cast<double>(nSamples) / static_cast<double>(fs);
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<int64_t>(durationSec * 1e6)));
  }
}