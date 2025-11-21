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
  };

  DualRtl(std::string type, uint32_t fc, uint32_t fs, std::string path, bool *saveIq,
          std::vector<double> gain, std::vector<std::string> serials, SyncConfig sync);

  void start() override;
  void stop() override;
  void process(IqData *buffer1, IqData *buffer2) override;
  void replay(IqData *buffer1, IqData *buffer2, std::string file, bool loop) override;

private:
  struct ChannelState
  {
    IqData *buffer = nullptr;
    std::atomic<size_t> dropSamples{0};
    std::complex<double> phaseCorrection{1.0, 0.0};
    bool applyPhaseCorrection = false;
  };

  std::vector<int> channelIndex;
  std::vector<int> gain;
  rtlsdr_dev_t *devs[2] = {nullptr, nullptr};
  SyncConfig syncConfig;
  std::array<size_t, 2> initialDrop{{0, 0}};
  std::complex<double> phaseCorrection{1.0, 0.0};
  double lastMeasuredSnrDb = 0.0;
  int64_t lastMeasuredOffset = 0;

  void check_status(int status, const std::string &message);
  bool measure_initial_offset();
  std::optional<std::pair<int64_t, std::complex<double>>> estimate_offset(const std::vector<std::complex<double>> &ref,
                                         const std::vector<std::complex<double>> &surv,
                                         double &snrDb,
                                         int32_t maxLag);
  static void callback(unsigned char *buf, uint32_t len, void *ctx);
};

#endif // DUAL_RTL_H
