/// @file DualRtl.h
/// @brief Capture driver for a pair of clock-shared RTL-SDRs with USB offset calibration.

#ifndef DUAL_RTL_H
#define DUAL_RTL_H

#include "capture/Source.h"
#include "data/IqData.h"

#include <array>
#include <atomic>
#include <complex>
#include <optional>
#include <string>
#include <vector>
#include <rtl-sdr.h>

class DualRtl : public Source
{
public:
  struct SyncConfig
  {
    bool enable = true;
    double seconds = 0.25;          ///< Duration of calibration capture.
    int32_t search = 65536;         ///< Max samples to search for correlation peak.
    double minSnrDb = 8.0;          ///< Warn if correlation peak SNR below this.
    uint32_t calibrationFc = 0;     ///< Optional frequency to retune for calibration.
    uint32_t recalibrateInterval = 0; ///< Re-run sync every N CPI cycles (0 = startup only).
  };

  /// @brief IQ imbalance correction parameters for a single channel.
  struct IqCorrection
  {
    double dcI = 0.0;              ///< Running DC offset for I channel.
    double dcQ = 0.0;              ///< Running DC offset for Q channel.
    double gainImbalance = 1.0;    ///< Q gain relative to I (ideal = 1.0).
    double phaseImbalance = 0.0;   ///< Q phase error in radians (ideal = 0.0).
    bool valid = false;            ///< True if calibration has been performed.
  };

  DualRtl(std::string type, uint32_t fc, uint32_t fs, std::string path, bool *saveIq,
          std::vector<double> gain, std::vector<std::string> serials, SyncConfig sync);

  void start() override;
  void stop() override;
  void process(IqData *buffer1, IqData *buffer2) override;
  void replay(IqData *buffer1, IqData *buffer2, std::string file, bool loop) override;
  bool get_sync_metrics(int64_t &offsetSamples, double &snrDb) const override;
  bool get_sample_drop_metrics(uint64_t &ch0, uint64_t &ch1) const override;

private:
  struct ChannelState
  {
    IqData *buffer = nullptr;
    DualRtl *owner = nullptr;
    uint8_t channelId = 0;
    std::atomic<size_t> dropSamples{0};
    std::complex<double> phaseCorrection{1.0, 0.0};
    bool applyPhaseCorrection = false;

    // DC offset removal (exponential moving average)
    double dcAlpha = 0.001;        ///< EMA smoothing factor for DC tracking.
    double dcI = 0.0;              ///< Running DC estimate for I.
    double dcQ = 0.0;              ///< Running DC estimate for Q.
    bool dcInitialised = false;    ///< True after first sample.

    // IQ imbalance correction
    double iqGainCorr = 1.0;       ///< Multiply Q by this to correct gain imbalance.
    double iqPhaseCorr = 0.0;      ///< sin(phase_error) for IQ orthogonality correction.
    bool applyIqCorrection = false;

    // Sample counting for drop detection
    std::atomic<uint64_t> totalSamples{0};
  };

  std::vector<int> channelIndex;
  std::vector<int> gain;
  rtlsdr_dev_t *devs[2] = {nullptr, nullptr};
  SyncConfig syncConfig;
  std::array<size_t, 2> initialDrop{{0, 0}};
  std::complex<double> phaseCorrection{1.0, 0.0};
  std::atomic<double> lastMeasuredSnrDb;
  std::atomic<int64_t> lastMeasuredOffset;
  std::array<std::atomic<uint64_t>, 2> sampleDropEstimate;

  // IQ imbalance correction per channel
  std::array<IqCorrection, 2> iqCorrections;

  void check_status(int status, const std::string &message);
  bool measure_initial_offset();
  void estimate_iq_imbalance(const std::vector<std::complex<double>> &samples,
                             IqCorrection &correction);
  std::optional<std::pair<int64_t, std::complex<double>>> estimate_offset(const std::vector<std::complex<double>> &ref,
                                         const std::vector<std::complex<double>> &surv,
                                         double &snrDb,
                                         int32_t maxLag);
  static void callback(unsigned char *buf, uint32_t len, void *ctx);
};

#endif // DUAL_RTL_H
