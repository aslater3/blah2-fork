#include "DualRtl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <cstring>
#include <fftw3.h>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
constexpr size_t kMaxChannels = 2;
}

DualRtl::DualRtl(std::string _type, uint32_t _fc, uint32_t _fs,
                 std::string _path, bool *_saveIq, std::vector<double> _gain,
                 std::vector<std::string> _serials, SyncConfig _sync)
    : Source(std::move(_type), _fc, _fs, std::move(_path), _saveIq),
      syncConfig(_sync)
{
  if (_gain.size() != kMaxChannels)
  {
    throw std::runtime_error("[dual-rtl] Exactly two gain entries are required.");
  }
  initialDrop.fill(0);

  // choose device indices either by provided serials or default order
  if (!_serials.empty())
  {
    if (_serials.size() != kMaxChannels)
    {
      throw std::runtime_error("[dual-rtl] Exactly two serial numbers are required when specifying serial mapping.");
    }
    int dev_count = rtlsdr_get_device_count();
    if (dev_count < static_cast<int>(kMaxChannels))
    {
      throw std::runtime_error("[dual-rtl] Not enough RTL-SDR devices detected.");
    }

    for (const auto &req_serial : _serials)
    {
      bool found = false;
      for (int idx = 0; idx < dev_count; ++idx)
      {
        char vendor[256] = {0}, product[256] = {0}, serialBuf[256] = {0};
        if (rtlsdr_get_device_usb_strings(idx, vendor, product, serialBuf) < 0)
        {
          continue;
        }
        if (req_serial == std::string(serialBuf))
        {
          channelIndex.push_back(idx);
          std::cout << "[dual-rtl] mapped serial " << req_serial << " -> device index " << idx << std::endl;
          found = true;
          break;
        }
      }
      if (!found)
      {
        throw std::runtime_error("[dual-rtl] Requested serial " + req_serial + " not found.");
      }
    }
  }
  else
  {
    channelIndex.push_back(0);
    channelIndex.push_back(1);
  }

  // query tuner gains to clamp user requests
  rtlsdr_dev_t *probe = nullptr;
  check_status(rtlsdr_open(&probe, channelIndex[0]), "[dual-rtl] Failed to open device for gain probe.");
  int nGains = rtlsdr_get_tuner_gains(probe, nullptr);
  check_status(nGains, "[dual-rtl] Failed to query tuner gain count.");
  std::vector<int> validGains(static_cast<size_t>(nGains));
  check_status(rtlsdr_get_tuner_gains(probe, validGains.data()), "[dual-rtl] Failed to read tuner gains.");
  check_status(rtlsdr_close(probe), "[dual-rtl] Failed to close probe device.");

  for (size_t i = 0; i < _gain.size(); ++i)
  {
    int requested = static_cast<int>(_gain[i] * 10);
    auto it = std::lower_bound(validGains.begin(), validGains.end(), requested);
    int applied = (it != validGains.end()) ? *it : validGains.back();
    gain.push_back(applied);
    std::cout << "[dual-rtl] gain channel " << i << ": requested " << requested
              << " -> applied " << applied << std::endl;
  }
}

void DualRtl::start()
{
  for (size_t i = 0; i < kMaxChannels; ++i)
  {
    char vendor[256] = {0}, product[256] = {0}, serialBuf[256] = {0};
    if (rtlsdr_get_device_usb_strings(channelIndex[i], vendor, product, serialBuf) == 0)
    {
      std::cout << "[dual-rtl] opening channel " << i << " (device index " << channelIndex[i]
                << ", serial=" << serialBuf << ")" << std::endl;
    }
    check_status(rtlsdr_open(&devs[i], channelIndex[i]), "[dual-rtl] Failed to open device.");
    check_status(rtlsdr_set_center_freq(devs[i], fc), "[dual-rtl] Failed to set center frequency.");
    check_status(rtlsdr_set_sample_rate(devs[i], fs), "[dual-rtl] Failed to set sample rate.");
    check_status(rtlsdr_set_dithering(devs[i], 0), "[dual-rtl] Failed to disable dithering.");
    check_status(rtlsdr_set_tuner_gain_mode(devs[i], 1), "[dual-rtl] Failed to disable AGC.");
    check_status(rtlsdr_set_tuner_gain(devs[i], gain[i]), "[dual-rtl] Failed to set gain.");
    check_status(rtlsdr_reset_buffer(devs[i]), "[dual-rtl] Failed to reset buffer.");
  }
}

void DualRtl::stop()
{
  for (size_t i = 0; i < kMaxChannels; ++i)
  {
    if (devs[i] != nullptr)
    {
      rtlsdr_cancel_async(devs[i]);
      rtlsdr_close(devs[i]);
      devs[i] = nullptr;
    }
  }
}

void DualRtl::process(IqData *buffer1, IqData *buffer2)
{
  if (!buffer1 || !buffer2)
  {
    throw std::runtime_error("[dual-rtl] Both reference and surveillance buffers are required.");
  }

  if (syncConfig.enable)
  {
    if (!measure_initial_offset())
    {
      std::cerr << "[dual-rtl] Sync calibration failed; proceeding without offset compensation." << std::endl;
      initialDrop[0] = 0;
      initialDrop[1] = 0;
    }
  }

  ChannelState channelState[2];
  channelState[0].buffer = buffer1;
  channelState[1].buffer = buffer2;
  channelState[0].dropSamples.store(initialDrop[0], std::memory_order_relaxed);
  channelState[1].dropSamples.store(initialDrop[1], std::memory_order_relaxed);

  std::vector<std::thread> threads;
  for (size_t i = 0; i < kMaxChannels; ++i)
  {
    // always reset before streaming in case calibration consumed data
    check_status(rtlsdr_reset_buffer(devs[i]), "[dual-rtl] Failed to reset buffer before streaming.");
    threads.emplace_back(rtlsdr_read_async, devs[i], callback, &channelState[i], 0, 16 * 16384);
  }

  for (auto &t : threads)
  {
    t.join();
  }
}

void DualRtl::replay(IqData *buffer1, IqData *buffer2, std::string file, bool loop)
{
  (void)buffer1;
  (void)buffer2;
  (void)file;
  (void)loop;
  std::cerr << "[dual-rtl] Replay not implemented." << std::endl;
}

void DualRtl::callback(unsigned char *buf, uint32_t len, void *ctx)
{
  auto *state = static_cast<ChannelState *>(ctx);
  if (!state || !state->buffer)
  {
    return;
  }

  int8_t *src = reinterpret_cast<int8_t *>(buf);
  size_t totalSamples = len / 2;
  size_t startIndex = 0;

  size_t dropRemaining = state->dropSamples.load(std::memory_order_relaxed);
  if (dropRemaining > 0)
  {
    size_t toDrop = std::min(dropRemaining, totalSamples);
    startIndex = toDrop;
    dropRemaining -= toDrop;
    state->dropSamples.store(dropRemaining, std::memory_order_relaxed);
  }

  state->buffer->lock();
  for (size_t i = startIndex; i < totalSamples; ++i)
  {
    double iqi = static_cast<double>(src[2 * i]);
    double iqq = static_cast<double>(src[2 * i + 1]);
    state->buffer->push_back({iqi, iqq});
  }
  state->buffer->unlock();
}

void DualRtl::check_status(int status, const std::string &message)
{
  if (status < 0)
  {
    throw std::runtime_error(message);
  }
}

bool DualRtl::measure_initial_offset()
{
  size_t samplesRequested = static_cast<size_t>(syncConfig.seconds * fs);
  if (samplesRequested == 0)
  {
    std::cerr << "[dual-rtl] Sync measurement duration is zero seconds." << std::endl;
    return false;
  }

  int32_t maxLag = std::max<int32_t>(1, syncConfig.search);
  samplesRequested = std::max(samplesRequested, static_cast<size_t>(2 * maxLag + 1));

  uint32_t measurementFc = syncConfig.calibrationFc != 0 ? syncConfig.calibrationFc : fc;
  bool retuned = measurementFc != fc;
  if (retuned)
  {
    for (auto *dev : devs)
    {
      check_status(rtlsdr_set_center_freq(dev, measurementFc), "[dual-rtl] Failed to set calibration frequency.");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::vector<unsigned char> raw0(samplesRequested * 2);
  std::vector<unsigned char> raw1(samplesRequested * 2);

  for (size_t i = 0; i < kMaxChannels; ++i)
  {
    check_status(rtlsdr_reset_buffer(devs[i]), "[dual-rtl] Failed to reset buffer before sync capture.");
  }

  auto capture_block = [](rtlsdr_dev_t *dev, std::vector<unsigned char> &dest, int &statusOut, int &bytesOut)
  {
    statusOut = rtlsdr_read_sync(dev, dest.data(), static_cast<int>(dest.size()), &bytesOut);
  };
  int status0 = 0, status1 = 0;
  int bytes0 = 0, bytes1 = 0;
  std::thread t0(capture_block, devs[0], std::ref(raw0), std::ref(status0), std::ref(bytes0));
  std::thread t1(capture_block, devs[1], std::ref(raw1), std::ref(status1), std::ref(bytes1));
  t0.join();
  t1.join();
  check_status(status0, "[dual-rtl] Failed to read sync block channel 0.");
  check_status(status1, "[dual-rtl] Failed to read sync block channel 1.");
  if (bytes0 < static_cast<int>(raw0.size()) || bytes1 < static_cast<int>(raw1.size()))
  {
    std::cerr << "[dual-rtl] Sync capture short read ("
              << bytes0 << "/" << raw0.size() << ", "
              << bytes1 << "/" << raw1.size() << ")." << std::endl;
    return false;
  }

  auto convert_samples = [](const std::vector<unsigned char> &raw)
  {
    std::vector<std::complex<double>> out(raw.size() / 2);
    const int8_t *ptr = reinterpret_cast<const int8_t *>(raw.data());
    for (size_t i = 0; i < out.size(); ++i)
    {
      double iqi = static_cast<double>(ptr[2 * i]);
      double iqq = static_cast<double>(ptr[2 * i + 1]);
      out[i] = {iqi, iqq};
    }
    std::complex<double> sum{0.0, 0.0};
    for (const auto &sample : out)
    {
      sum += sample;
    }
    std::complex<double> mean = sum / static_cast<double>(out.size());
    for (auto &sample : out)
    {
      sample -= mean;
    }
    return out;
  };

  auto ref = convert_samples(raw0);
  auto surv = convert_samples(raw1);

  if (retuned)
  {
    for (auto *dev : devs)
    {
      check_status(rtlsdr_set_center_freq(dev, fc), "[dual-rtl] Failed to restore center frequency.");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  double snrDb = 0.0;
  auto lagOpt = estimate_offset(ref, surv, snrDb, maxLag);
  if (!lagOpt)
  {
    std::cerr << "[dual-rtl] Unable to compute correlation-based offset." << std::endl;
    return false;
  }

  lastMeasuredOffset = *lagOpt;
  lastMeasuredSnrDb = snrDb;

  if (std::abs(lastMeasuredOffset) > maxLag)
  {
    std::cerr << "[dual-rtl] Measured offset exceeds search window." << std::endl;
    return false;
  }

  if (lastMeasuredOffset < 0)
  {
    initialDrop[0] = static_cast<size_t>(-lastMeasuredOffset);
    initialDrop[1] = 0;
  }
  else
  {
    initialDrop[0] = 0;
    initialDrop[1] = static_cast<size_t>(lastMeasuredOffset);
  }

  double offsetUs = (static_cast<double>(lastMeasuredOffset) / static_cast<double>(fs)) * 1e6;
  std::cout << "[dual-rtl] Measured initial offset " << lastMeasuredOffset << " samples ("
            << offsetUs << " us) SNR=" << snrDb << " dB" << std::endl;
  if (snrDb < syncConfig.minSnrDb)
  {
    std::cerr << "[dual-rtl] Warning: correlation peak SNR " << snrDb
              << " dB below threshold " << syncConfig.minSnrDb << " dB." << std::endl;
  }

  return true;
}

std::optional<int64_t> DualRtl::estimate_offset(const std::vector<std::complex<double>> &ref,
                                                const std::vector<std::complex<double>> &surv,
                                                double &snrDb,
                                                int32_t maxLag)
{
  size_t n = std::min(ref.size(), surv.size());
  if (n == 0)
  {
    return std::nullopt;
  }

  size_t nfft = 1;
  while (nfft < 2 * n)
  {
    nfft <<= 1;
  }

  fftw_complex *X = reinterpret_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * nfft));
  fftw_complex *Y = reinterpret_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * nfft));
  fftw_complex *Z = reinterpret_cast<fftw_complex *>(fftw_malloc(sizeof(fftw_complex) * nfft));
  if (!X || !Y || !Z)
  {
    if (X)
      fftw_free(X);
    if (Y)
      fftw_free(Y);
    if (Z)
      fftw_free(Z);
    return std::nullopt;
  }

  for (size_t i = 0; i < nfft; ++i)
  {
    if (i < n)
    {
      X[i][0] = ref[i].real();
      X[i][1] = ref[i].imag();
      Y[i][0] = surv[i].real();
      Y[i][1] = surv[i].imag();
    }
    else
    {
      X[i][0] = X[i][1] = 0.0;
      Y[i][0] = Y[i][1] = 0.0;
    }
  }

  auto planX = fftw_plan_dft_1d(static_cast<int>(nfft), X, X, FFTW_FORWARD, FFTW_ESTIMATE);
  auto planY = fftw_plan_dft_1d(static_cast<int>(nfft), Y, Y, FFTW_FORWARD, FFTW_ESTIMATE);
  auto planZ = fftw_plan_dft_1d(static_cast<int>(nfft), Z, Z, FFTW_BACKWARD, FFTW_ESTIMATE);
  fftw_execute(planX);
  fftw_execute(planY);

  for (size_t k = 0; k < nfft; ++k)
  {
    double xr = X[k][0];
    double xi = X[k][1];
    double yr = Y[k][0];
    double yi = Y[k][1];
    // cross spectrum X * conj(Y)
    Z[k][0] = xr * yr + xi * yi;
    Z[k][1] = -xr * yi + xi * yr;
  }

  fftw_execute(planZ);

  int64_t bestLag = 0;
  double bestMag = -1.0;
  double noiseSum = 0.0;
  size_t considered = 0;

  int64_t lagLimit = std::min<int64_t>(maxLag, static_cast<int64_t>(n) - 1);
  for (size_t idx = 0; idx < nfft; ++idx)
  {
    int64_t lag = static_cast<int64_t>(idx);
    if (lag > static_cast<int64_t>(nfft) / 2)
    {
      lag -= static_cast<int64_t>(nfft);
    }
    if (std::llabs(lag) > lagLimit)
    {
      continue;
    }
    double mag = std::hypot(Z[idx][0], Z[idx][1]) / static_cast<double>(nfft);
    if (mag > bestMag)
    {
      if (bestMag >= 0.0)
      {
        noiseSum += bestMag;
      }
      bestMag = mag;
      bestLag = lag;
    }
    else
    {
      noiseSum += mag;
    }
    considered++;
  }

  fftw_destroy_plan(planX);
  fftw_destroy_plan(planY);
  fftw_destroy_plan(planZ);
  fftw_free(X);
  fftw_free(Y);
  fftw_free(Z);

  if (considered < 2 || bestMag <= 0.0)
  {
    return std::nullopt;
  }

  double avgNoise = noiseSum / static_cast<double>(considered - 1);
  snrDb = 20.0 * std::log10(bestMag / (avgNoise + 1e-9));
  return bestLag;
}
