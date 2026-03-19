#include "LibreSdr.h"

#include <iostream>
#include <complex>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <thread>
#include <chrono>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Errors.hpp>

// constructor
LibreSdr::LibreSdr(std::string _type, uint32_t _fc, uint32_t _fs,
                   std::string _path, bool *_saveIq,
                   std::vector<double> _gain,
                   std::vector<std::string> _antenna,
                   double _bandwidth,
                   std::string _deviceArgs)
    : Source(_type, _fc, _fs, _path, _saveIq),
      gain(std::move(_gain)),
      antenna(std::move(_antenna)),
      bandwidth(_bandwidth),
      deviceArgs(std::move(_deviceArgs)),
      soapyDevice(nullptr),
      rxStream(nullptr)
{
}

LibreSdr::~LibreSdr()
{
  stop();
}

void LibreSdr::start()
{
  // Make the SoapySDR device
  SoapySDR::Kwargs args = SoapySDR::KwargsFromString(deviceArgs);
  soapyDevice = SoapySDR::Device::make(args);
  if (soapyDevice == nullptr)
  {
    std::cerr << "Error: LibreSDR - SoapySDR::Device::make() failed." << std::endl;
    return;
  }

  // Configure RX channel 0 (reference)
  soapyDevice->setSampleRate(SOAPY_SDR_RX, 0, static_cast<double>(fs));
  soapyDevice->setFrequency(SOAPY_SDR_RX, 0, static_cast<double>(fc));
  if (gain.size() > 0)
  {
    soapyDevice->setGain(SOAPY_SDR_RX, 0, gain[0]);
  }
  if (antenna.size() > 0 && !antenna[0].empty())
  {
    soapyDevice->setAntenna(SOAPY_SDR_RX, 0, antenna[0]);
  }
  if (bandwidth > 0)
  {
    soapyDevice->setBandwidth(SOAPY_SDR_RX, 0, bandwidth);
  }

  // Configure RX channel 1 (surveillance)
  soapyDevice->setSampleRate(SOAPY_SDR_RX, 1, static_cast<double>(fs));
  soapyDevice->setFrequency(SOAPY_SDR_RX, 1, static_cast<double>(fc));
  if (gain.size() > 1)
  {
    soapyDevice->setGain(SOAPY_SDR_RX, 1, gain[1]);
  }
  if (antenna.size() > 1 && !antenna[1].empty())
  {
    soapyDevice->setAntenna(SOAPY_SDR_RX, 1, antenna[1]);
  }
  if (bandwidth > 0)
  {
    soapyDevice->setBandwidth(SOAPY_SDR_RX, 1, bandwidth);
  }

  // Print actual configuration
  std::cout << "LibreSDR RX0 rate: "
            << soapyDevice->getSampleRate(SOAPY_SDR_RX, 0) << " Hz" << std::endl;
  std::cout << "LibreSDR RX0 freq: "
            << soapyDevice->getFrequency(SOAPY_SDR_RX, 0) << " Hz" << std::endl;
  std::cout << "LibreSDR RX0 gain: "
            << soapyDevice->getGain(SOAPY_SDR_RX, 0) << " dB" << std::endl;
  std::cout << "LibreSDR RX0 antenna: "
            << soapyDevice->getAntenna(SOAPY_SDR_RX, 0) << std::endl;
  std::cout << "LibreSDR RX1 rate: "
            << soapyDevice->getSampleRate(SOAPY_SDR_RX, 1) << " Hz" << std::endl;
  std::cout << "LibreSDR RX1 freq: "
            << soapyDevice->getFrequency(SOAPY_SDR_RX, 1) << " Hz" << std::endl;
  std::cout << "LibreSDR RX1 gain: "
            << soapyDevice->getGain(SOAPY_SDR_RX, 1) << " dB" << std::endl;
  std::cout << "LibreSDR RX1 antenna: "
            << soapyDevice->getAntenna(SOAPY_SDR_RX, 1) << std::endl;

  // Setup a dual-channel RX stream (CF32 format)
  std::vector<size_t> channels = {0, 1};
  rxStream = soapyDevice->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, channels);
  if (rxStream == nullptr)
  {
    std::cerr << "Error: LibreSDR - setupStream() failed." << std::endl;
    return;
  }

  // Activate the stream
  int ret = soapyDevice->activateStream(rxStream);
  if (ret != 0)
  {
    std::cerr << "Error: LibreSDR - activateStream() returned "
              << SoapySDR::errToStr(ret) << std::endl;
  }

  std::cout << "LibreSDR capture started." << std::endl;
}

void LibreSdr::stop()
{
  if (soapyDevice != nullptr && rxStream != nullptr)
  {
    soapyDevice->deactivateStream(rxStream);
    soapyDevice->closeStream(rxStream);
    rxStream = nullptr;
  }
  if (soapyDevice != nullptr)
  {
    SoapySDR::Device::unmake(soapyDevice);
    soapyDevice = nullptr;
  }
}

void LibreSdr::process(IqData *buffer1, IqData *buffer2)
{
  if (soapyDevice == nullptr || rxStream == nullptr)
  {
    std::cerr << "Error: LibreSDR - device not started." << std::endl;
    return;
  }

  // Get the MTU (maximum transfer unit) for this stream
  size_t mtu = soapyDevice->getStreamMTU(rxStream);
  if (mtu == 0)
  {
    mtu = 1024; // sensible fallback
  }

  // Allocate per-channel receive buffers (CF32 = std::complex<float>)
  std::vector<std::complex<float>> rxBuf0(mtu);
  std::vector<std::complex<float>> rxBuf1(mtu);

  // SoapySDR readStream expects an array of void* pointers, one per channel
  void *buffs[2] = {rxBuf0.data(), rxBuf1.data()};

  int flags = 0;
  long long timeNs = 0;

  while (true)
  {
    // Read samples from both channels simultaneously
    int ret = soapyDevice->readStream(rxStream, buffs, mtu, flags, timeNs);

    if (ret < 0)
    {
      // Handle overflow — log it but keep going
      if (ret == SOAPY_SDR_OVERFLOW)
      {
        std::cerr << "Warning: LibreSDR overflow detected." << std::endl;
        continue;
      }
      // Handle timeout — retry
      if (ret == SOAPY_SDR_TIMEOUT)
      {
        continue;
      }
      // Other error — log and break
      std::cerr << "Error: LibreSDR readStream returned "
                << SoapySDR::errToStr(ret) << std::endl;
      break;
    }

    size_t nReceived = static_cast<size_t>(ret);

    // Push samples into the IqData ring buffers
    buffer1->lock();
    buffer2->lock();
    for (size_t i = 0; i < nReceived; i++)
    {
      buffer1->push_back({static_cast<double>(rxBuf0[i].real()),
                          static_cast<double>(rxBuf0[i].imag())});
      buffer2->push_back({static_cast<double>(rxBuf1[i].real()),
                          static_cast<double>(rxBuf1[i].imag())});
    }
    buffer1->unlock();
    buffer2->unlock();

    // Save IQ data to file if enabled
    if (*saveIq)
    {
      saveIqFile.write(reinterpret_cast<const char *>(rxBuf0.data()),
                       nReceived * sizeof(std::complex<float>));
      saveIqFile.write(reinterpret_cast<const char *>(rxBuf1.data()),
                       nReceived * sizeof(std::complex<float>));
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

  // Replay file format: interleaved CF32 samples, channel 0 then channel 1
  // per block of nSamples.
  const size_t blockSize = 1024;
  std::vector<std::complex<float>> rxBuf0(blockSize);
  std::vector<std::complex<float>> rxBuf1(blockSize);

  while (true)
  {
    // Read channel 0 block
    inFile.read(reinterpret_cast<char *>(rxBuf0.data()),
                blockSize * sizeof(std::complex<float>));
    std::streamsize bytesRead0 = inFile.gcount();
    size_t samplesRead0 = bytesRead0 / sizeof(std::complex<float>);

    // Read channel 1 block
    inFile.read(reinterpret_cast<char *>(rxBuf1.data()),
                blockSize * sizeof(std::complex<float>));
    std::streamsize bytesRead1 = inFile.gcount();
    size_t samplesRead1 = bytesRead1 / sizeof(std::complex<float>);

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
      buffer1->push_back({static_cast<double>(rxBuf0[i].real()),
                          static_cast<double>(rxBuf0[i].imag())});
      buffer2->push_back({static_cast<double>(rxBuf1[i].real()),
                          static_cast<double>(rxBuf1[i].imag())});
    }
    buffer1->unlock();
    buffer2->unlock();

    // Throttle replay to approximate real-time
    double durationSec = static_cast<double>(nSamples) / static_cast<double>(fs);
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<int64_t>(durationSec * 1e6)));
  }
}