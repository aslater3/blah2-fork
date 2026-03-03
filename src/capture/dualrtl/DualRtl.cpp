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
#include <fstream>
#include <iomanip>

namespace
{
constexpr size_t kMaxChannels = 2;
}

DualRtl::DualRtl(std::string _type, uint32_t _fc, uint32_t _fs,
                 std::string _path, bool *_saveIq, std::vector<double> _gain,
                 std::vector<std::string> _serials, SyncConfig _sync)
    : Source(std::move(_type), _fc, _fs, std::move(_path), _saveIq),
      syncConfig(_sync),
      lastMeasuredSnrDb(0.0),
      lastMeasuredOffset(0)
{
  if (_gain.size() != kMaxChannels)
  {
    throw std::runtime_error("[dual-rtl] Exactly two gain entries are required.");
  }
  initialDrop.fill(0);
  sampleDropEstimate[0].store(0, std::memory_order_relaxed);
  sampleDropEstimate[1].store(0, std::memory_order_relaxed);

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

  // Track CPI cycles for periodic recalibration
  uint32_t cpiCount = 0;
  bool needsRecalibration = false;

  ChannelState channelState[2];
  channelState[0].buffer = buffer1;
  channelState[0].owner = this;
  channelState[0].channelId = 0;
  channelState[1].buffer = buffer2;
  channelState[1].owner = this;
  channelState[1].channelId = 1;
  channelState[0].dropSamples.store(initialDrop[0], std::memory_order_relaxed);
  channelState[1].dropSamples.store(initialDrop[1], std::memory_order_relaxed);

  // Apply phase correction to the second channel (surveillance)
  channelState[1].phaseCorrection = phaseCorrection;
  channelState[1].applyPhaseCorrection = true;

  // Apply IQ imbalance corrections if estimated during calibration
  for (size_t i = 0; i < kMaxChannels; ++i)
  {
    if (iqCorrections[i].valid)
    {
      channelState[i].iqGainCorr = 1.0 / iqCorrections[i].gainImbalance;
      channelState[i].iqPhaseCorr = std::sin(iqCorrections[i].phaseImbalance);
      channelState[i].applyIqCorrection = true;
      std::cout << "[dual-rtl] IQ correction ch" << i
                << ": gain=" << channelState[i].iqGainCorr
                << " phase=" << iqCorrections[i].phaseImbalance << " rad" << std::endl;
    }
  }

  // Lambda to run one streaming epoch (runs until buffers are full enough for
  // the processing thread to consume a CPI, then we check if recalibration
  // is needed).
  auto run_streaming = [&]()
  {
    for (size_t i = 0; i < kMaxChannels; ++i)
    {
      check_status(rtlsdr_reset_buffer(devs[i]), "[dual-rtl] Failed to reset buffer before streaming.");
    }

    std::vector<std::thread> threads;
    for (size_t i = 0; i < kMaxChannels; ++i)
    {
      threads.emplace_back(rtlsdr_read_async, devs[i], callback, &channelState[i], 0, 16 * 16384);
    }

    int64_t lastSampleDivergence = 0;
    uint32_t divergenceBreachCount = 0;
    // Async callback scheduling naturally creates short-lived count deltas.
    // Only trigger recalibration if divergence is both large and persistent.
    // Use conservative thresholds to avoid recalibration spirals when
    // processing is slow (which causes apparent divergence via buffer overflow).
    constexpr int64_t kDivergenceTriggerSamples = 32 * 16384;
    constexpr uint32_t kDivergenceBreachLimit = 50; // ~5 s at 100 ms polling
    // Monitor for recalibration trigger or sample drops.
    // The async reads run indefinitely; we periodically check.
    while (true)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // Check for sample drop divergence between channels
      uint64_t count0 = channelState[0].totalSamples.load(std::memory_order_relaxed);
      uint64_t count1 = channelState[1].totalSamples.load(std::memory_order_relaxed);
      int64_t sampleDivergence = static_cast<int64_t>(count0) - static_cast<int64_t>(count1);
      int64_t deltaDivergence = sampleDivergence - lastSampleDivergence;
      if (deltaDivergence > 0)
      {
        sampleDropEstimate[1].fetch_add(static_cast<uint64_t>(deltaDivergence), std::memory_order_relaxed);
      }
      else if (deltaDivergence < 0)
      {
        sampleDropEstimate[0].fetch_add(static_cast<uint64_t>(-deltaDivergence), std::memory_order_relaxed);
      }
      lastSampleDivergence = sampleDivergence;

      if (std::abs(sampleDivergence) > kDivergenceTriggerSamples)
      {
        divergenceBreachCount++;
        if (divergenceBreachCount >= kDivergenceBreachLimit)
        {
          std::cerr << "[dual-rtl] WARNING: Persistent sample count divergence detected: "
                    << sampleDivergence << " samples (ch0=" << count0
                    << " ch1=" << count1 << "). Triggering recalibration." << std::endl;
          needsRecalibration = true;
          divergenceBreachCount = 0;
        }
      }
      else
      {
        divergenceBreachCount = 0;
      }

      // Check if periodic recalibration is due.
      // We estimate CPI count from the total samples processed.
      // A CPI at fs=2.4MHz and cpi=1.0s is ~2.4M samples.
      uint64_t minSamples = std::min(count0, count1);
      uint32_t estimatedCpi = static_cast<uint32_t>(minSamples / fs);
      if (syncConfig.recalibrateInterval > 0 && estimatedCpi > cpiCount)
      {
        cpiCount = estimatedCpi;
        if (cpiCount > 0 && (cpiCount % syncConfig.recalibrateInterval) == 0)
        {
          std::cout << "[dual-rtl] Periodic recalibration triggered at CPI #" << cpiCount << std::endl;
          needsRecalibration = true;
        }
      }

      if (needsRecalibration)
      {
        // Cancel async reads so we can do synchronous calibration
        for (size_t i = 0; i < kMaxChannels; ++i)
        {
          rtlsdr_cancel_async(devs[i]);
        }
        break;
      }
    }

    for (auto &t : threads)
    {
      t.join();
    }
  };

  // Main loop: stream, recalibrate if needed, repeat
  while (true)
  {
    run_streaming();

    if (!needsRecalibration)
    {
      break; // Normal exit (shouldn't happen in practice as async reads run forever)
    }

    // Perform recalibration
    needsRecalibration = false;

    std::cout << "[dual-rtl] Running recalibration..." << std::endl;
    if (measure_initial_offset())
    {
      // Update channel state with new corrections
      channelState[0].dropSamples.store(initialDrop[0], std::memory_order_relaxed);
      channelState[1].dropSamples.store(initialDrop[1], std::memory_order_relaxed);
      channelState[1].phaseCorrection = phaseCorrection;

      // Update IQ corrections
      for (size_t i = 0; i < kMaxChannels; ++i)
      {
        if (iqCorrections[i].valid)
        {
          channelState[i].iqGainCorr = 1.0 / iqCorrections[i].gainImbalance;
          channelState[i].iqPhaseCorr = std::sin(iqCorrections[i].phaseImbalance);
          channelState[i].applyIqCorrection = true;
        }
      }

      // Reset sample counters
      channelState[0].totalSamples.store(0, std::memory_order_relaxed);
      channelState[1].totalSamples.store(0, std::memory_order_relaxed);
      cpiCount = 0;

      std::cout << "[dual-rtl] Recalibration complete. Resuming streaming." << std::endl;
    }
    else
    {
      std::cerr << "[dual-rtl] Recalibration failed; continuing with previous corrections." << std::endl;
    }
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

bool DualRtl::get_sync_metrics(int64_t &offsetSamples, double &snrDb) const
{
  offsetSamples = lastMeasuredOffset.load(std::memory_order_relaxed);
  snrDb = lastMeasuredSnrDb.load(std::memory_order_relaxed);
  return syncConfig.enable;
}

bool DualRtl::get_sample_drop_metrics(uint64_t &ch0, uint64_t &ch1) const
{
  ch0 = sampleDropEstimate[0].load(std::memory_order_relaxed);
  ch1 = sampleDropEstimate[1].load(std::memory_order_relaxed);
  return true;
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

  // Drop samples for initial alignment
  size_t dropRemaining = state->dropSamples.load(std::memory_order_relaxed);
  if (dropRemaining > 0)
  {
    size_t toDrop = std::min(dropRemaining, totalSamples);
    startIndex = toDrop;
    dropRemaining -= toDrop;
    state->dropSamples.store(dropRemaining, std::memory_order_relaxed);
    if (state->owner)
    {
      state->owner->sampleDropEstimate[state->channelId].fetch_add(
        static_cast<uint64_t>(toDrop), std::memory_order_relaxed);
    }
  }

  // Update total sample counter for drop detection
  state->totalSamples.fetch_add(totalSamples - startIndex, std::memory_order_relaxed);

  state->buffer->lock();

  for (size_t i = startIndex; i < totalSamples; ++i)
  {
    double iqi = static_cast<double>(src[2 * i]);
    double iqq = static_cast<double>(src[2 * i + 1]);

    // DC offset removal using exponential moving average
    if (!state->dcInitialised)
    {
      state->dcI = iqi;
      state->dcQ = iqq;
      state->dcInitialised = true;
    }
    else
    {
      state->dcI += state->dcAlpha * (iqi - state->dcI);
      state->dcQ += state->dcAlpha * (iqq - state->dcQ);
    }
    iqi -= state->dcI;
    iqq -= state->dcQ;

    // IQ imbalance correction
    // Corrects gain imbalance and quadrature error using:
    //   I_corrected = I
    //   Q_corrected = Q * gainCorr - I * phaseCorr
    if (state->applyIqCorrection)
    {
      double correctedQ = iqq * state->iqGainCorr - iqi * state->iqPhaseCorr;
      iqq = correctedQ;
    }

    // Phase correction (inter-channel alignment)
    if (state->applyPhaseCorrection)
    {
      std::complex<double> s(iqi, iqq);
      s *= state->phaseCorrection;
      state->buffer->push_back({s.real(), s.imag()});
    }
    else
    {
      state->buffer->push_back({iqi, iqq});
    }
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

void DualRtl::estimate_iq_imbalance(const std::vector<std::complex<double>> &samples,
                                     IqCorrection &correction)
{
  if (samples.size() < 1000)
  {
    return;
  }

  // Estimate DC offset
  double sumI = 0.0, sumQ = 0.0;
  for (const auto &s : samples)
  {
    sumI += s.real();
    sumQ += s.imag();
  }
  correction.dcI = sumI / static_cast<double>(samples.size());
  correction.dcQ = sumQ / static_cast<double>(samples.size());

  // Estimate gain and phase imbalance using second-order statistics
  // E[I^2], E[Q^2], E[I*Q] after DC removal
  double sumII = 0.0, sumQQ = 0.0, sumIQ = 0.0;
  for (const auto &s : samples)
  {
    double i = s.real() - correction.dcI;
    double q = s.imag() - correction.dcQ;
    sumII += i * i;
    sumQQ += q * q;
    sumIQ += i * q;
  }
  double n = static_cast<double>(samples.size());
  double eII = sumII / n;
  double eQQ = sumQQ / n;
  double eIQ = sumIQ / n;

  // Gain imbalance: ratio of RMS amplitudes
  if (eII > 0.0)
  {
    correction.gainImbalance = std::sqrt(eQQ / eII);
  }
  else
  {
    correction.gainImbalance = 1.0;
  }

  // Phase imbalance: correlation coefficient indicates quadrature error
  double denominator = std::sqrt(eII * eQQ);
  if (denominator > 0.0)
  {
    double rho = eIQ / denominator;
    // rho ≈ sin(phase_error) for small errors
    correction.phaseImbalance = std::asin(std::clamp(rho, -1.0, 1.0));
  }
  else
  {
    correction.phaseImbalance = 0.0;
  }

  correction.valid = true;

  std::cout << "[dual-rtl] IQ imbalance estimate:"
            << " dcI=" << correction.dcI
            << " dcQ=" << correction.dcQ
            << " gain=" << correction.gainImbalance
            << " phase=" << correction.phaseImbalance << " rad"
            << std::endl;
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

  // align to RTL-SDR transfer size (multiple of 16384 samples).
  constexpr size_t kSampleAlign = 16384;
  if (samplesRequested % kSampleAlign != 0)
  {
    samplesRequested = ((samplesRequested / kSampleAlign) + 1) * kSampleAlign;
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
  if (status0 < 0)
  {
    std::cerr << "[dual-rtl] Failed to read sync block channel 0: status " << status0
              << " (" << strerror(-status0) << ")." << std::endl;
    return false;
  }
  if (status1 < 0)
  {
    std::cerr << "[dual-rtl] Failed to read sync block channel 1: status " << status1
              << " (" << strerror(-status1) << ")." << std::endl;
    return false;
  }
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

  // Estimate IQ imbalance for each channel from calibration data
  // (use raw samples before DC removal for accurate DC estimation)
  {
    auto raw_convert = [](const std::vector<unsigned char> &raw)
    {
      std::vector<std::complex<double>> out(raw.size() / 2);
      const int8_t *ptr = reinterpret_cast<const int8_t *>(raw.data());
      for (size_t i = 0; i < out.size(); ++i)
      {
        out[i] = {static_cast<double>(ptr[2 * i]), static_cast<double>(ptr[2 * i + 1])};
      }
      return out;
    };
    auto rawRef = raw_convert(raw0);
    auto rawSurv = raw_convert(raw1);
    estimate_iq_imbalance(rawRef, iqCorrections[0]);
    estimate_iq_imbalance(rawSurv, iqCorrections[1]);
  }

  double snrDb = 0.0;
  auto resultOpt = estimate_offset(ref, surv, snrDb, maxLag);
  if (!resultOpt)
  {
    std::cerr << "[dual-rtl] Unable to compute correlation-based offset." << std::endl;
    return false;
  }

  lastMeasuredOffset.store(resultOpt->first, std::memory_order_relaxed);
  phaseCorrection = resultOpt->second;
  lastMeasuredSnrDb.store(snrDb, std::memory_order_relaxed);

  int64_t measuredOffset = lastMeasuredOffset.load(std::memory_order_relaxed);
  double measuredSnrDb = lastMeasuredSnrDb.load(std::memory_order_relaxed);

  if (std::abs(measuredOffset) > maxLag)
  {
    std::cerr << "[dual-rtl] Measured offset exceeds search window." << std::endl;
    return false;
  }

  if (measuredOffset < 0)
  {
    initialDrop[0] = static_cast<size_t>(-measuredOffset);
    initialDrop[1] = 0;
  }
  else
  {
    initialDrop[0] = 0;
    initialDrop[1] = static_cast<size_t>(measuredOffset);
  }

  double offsetUs = (static_cast<double>(measuredOffset) / static_cast<double>(fs)) * 1e6;
  std::cout << "[dual-rtl] Measured initial offset " << measuredOffset << " samples ("
            << offsetUs << " us) Phase=" << std::arg(phaseCorrection) << " rad SNR=" << measuredSnrDb << " dB" << std::endl;
  if (measuredSnrDb < syncConfig.minSnrDb)
  {
    std::cerr << "[dual-rtl] Warning: correlation peak SNR " << measuredSnrDb
              << " dB below threshold " << syncConfig.minSnrDb << " dB." << std::endl;
  }

  // Write calibration data to file
  std::ofstream calFile("calibration.json");
  if (calFile.is_open())
  {
    calFile << "{\n";
    calFile << "  \"offset_samples\": " << measuredOffset << ",\n";
    calFile << "  \"offset_us\": " << offsetUs << ",\n";
    calFile << "  \"phase_rad\": " << std::arg(phaseCorrection) << ",\n";
    calFile << "  \"snr_db\": " << measuredSnrDb << ",\n";
    calFile << "  \"iq_correction_ch0\": {\n";
    calFile << "    \"dc_i\": " << iqCorrections[0].dcI << ",\n";
    calFile << "    \"dc_q\": " << iqCorrections[0].dcQ << ",\n";
    calFile << "    \"gain_imbalance\": " << iqCorrections[0].gainImbalance << ",\n";
    calFile << "    \"phase_imbalance_rad\": " << iqCorrections[0].phaseImbalance << "\n";
    calFile << "  },\n";
    calFile << "  \"iq_correction_ch1\": {\n";
    calFile << "    \"dc_i\": " << iqCorrections[1].dcI << ",\n";
    calFile << "    \"dc_q\": " << iqCorrections[1].dcQ << ",\n";
    calFile << "    \"gain_imbalance\": " << iqCorrections[1].gainImbalance << ",\n";
    calFile << "    \"phase_imbalance_rad\": " << iqCorrections[1].phaseImbalance << "\n";
    calFile << "  }\n";
    calFile << "}\n";
    calFile.close();
  }

  return true;
}

std::optional<std::pair<int64_t, std::complex<double>>> DualRtl::estimate_offset(const std::vector<std::complex<double>> &ref,
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
  std::complex<double> bestVal(0.0, 0.0);
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
      bestVal = {Z[idx][0], Z[idx][1]};
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
  
  // Calculate phase correction
  // Z = X * conj(Y). Peak phase is phase(X) - phase(Y).
  // We want to rotate Y to match X, so we multiply Y by exp(j * (phase(X) - phase(Y))).
  // This is exactly exp(j * arg(Z_peak)).
  std::complex<double> correction = std::exp(std::complex<double>(0, std::arg(bestVal)));

  return std::make_pair(bestLag, correction);
}
