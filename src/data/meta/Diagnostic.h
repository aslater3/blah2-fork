/// @file Diagnostic.h
/// @class Diagnostic
/// @brief Per-CPI diagnostic logging and detection persistence tracking.
/// @details Tracks recurring detections across CPIs to identify artefacts vs real targets.
/// Logs per-CPI summary including noise floor, clutter suppression, detection counts,
/// and recurring detection analysis.

#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H

#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <fstream>

struct PersistentDetection
{
  double delay;
  double doppler;
  double snr;
  uint32_t persistence;     // consecutive CPIs at same location
  uint64_t firstSeen;       // timestamp of first appearance
  uint64_t lastSeen;        // timestamp of most recent appearance
};

struct DetectionState
{
  double delay;
  double doppler;
  double snr;
  uint32_t persistence;
  std::string status; // NEW, RECURRING, MOVING
};

struct CfarRowMetric
{
  double doppler;
  double noisePower_dB;
  double threshold_dB;
  uint32_t nDetections;
};

struct CpiDiagnostic
{
  uint64_t timestamp;
  double noisePower_dB;
  double maxPower_dB;
  uint32_t nDetections_raw;
  uint32_t nDetections_centroid;
  uint32_t nDetections_final;
  double clutterPower_before_dB;  // power at delay=0 before clutter filter
  double clutterPower_after_dB;   // power at delay=0 after clutter filter
  double clutterSuppression_dB;
  int64_t syncOffset_samples;
  double syncSNR_dB;
  bool syncValid;
  uint64_t sampleDrops_ch0;
  uint64_t sampleDrops_ch1;
  std::vector<CfarRowMetric> cfarRows;
  std::vector<DetectionState> detectionStates;
  uint32_t nRecurring;
  std::vector<PersistentDetection> recurringDetections;
};

class Diagnostic
{
public:
  /// @brief Constructor.
  /// @param savePath Base path for diagnostic log files.
  /// @param delayTolerance Tolerance in delay bins for matching detections across CPIs.
  /// @param dopplerTolerance Tolerance in Hz for matching detections across CPIs.
  /// @param persistenceThreshold Minimum CPIs to flag as "recurring".
  Diagnostic(const std::string &savePath, double delayTolerance = 1.5,
             double dopplerTolerance = 2.0, uint32_t persistenceThreshold = 5);

  ~Diagnostic();

  /// @brief Record clutter power before and after filtering.
  /// @param before Power at delay=0 before clutter filter (linear).
  /// @param after Power at delay=0 after clutter filter (linear).
  void set_clutter_power(double before, double after);

  /// @brief Record detection counts at each processing stage.
  void set_detection_counts(uint32_t raw, uint32_t centroid, uint32_t final_count);

  /// @brief Record noise and max power from the ambiguity map.
  void set_map_metrics(double noisePower, double maxPower);

  /// @brief Record sync calibration metrics.
  void set_sync_metrics(int64_t offsetSamples, double snrDb, bool valid);

  /// @brief Record dropped sample counters for both channels.
  void set_sample_drops(uint64_t ch0, uint64_t ch1);

  /// @brief Record per-Doppler-row CFAR diagnostics.
  void set_cfar_rows(const std::vector<double> &doppler,
                     const std::vector<double> &noisePower,
                     const std::vector<double> &threshold,
                     const std::vector<uint32_t> &nDetections);

  /// @brief Update persistence tracking with current CPI detections.
  /// @param delay Vector of detection delays.
  /// @param doppler Vector of detection Dopplers.
  /// @param snr Vector of detection SNRs.
  /// @param timestamp Current CPI timestamp.
  void update_persistence(const std::vector<double> &delay,
                          const std::vector<double> &doppler,
                          const std::vector<double> &snr,
                          uint64_t timestamp);

  /// @brief Write the per-CPI diagnostic summary to the log file.
  /// @param timestamp Current CPI timestamp.
  void log_cpi(uint64_t timestamp);

  /// @brief Write a zero-Doppler-row snapshot (before and after clutter filtering).
  void log_zero_doppler_row(uint64_t timestamp,
                            const std::vector<double> &before_dB,
                            const std::vector<double> &after_dB);

  /// @brief Get JSON string of current recurring detections for API output.
  std::string recurring_to_json() const;

  /// @brief Get the current CPI diagnostic data.
  CpiDiagnostic get_current() const;

private:
  std::string savePath;
  std::ofstream logFile;
  std::ofstream zeroDopplerFile;
  double delayTolerance;
  double dopplerTolerance;
  uint32_t persistenceThreshold;

  CpiDiagnostic current;

  // persistence tracking: key is quantised (delay, doppler)
  struct DetectionKey
  {
    int delayBin;
    int dopplerBin;  // quantised to 1 Hz
    bool operator<(const DetectionKey &o) const
    {
      if (delayBin != o.delayBin) return delayBin < o.delayBin;
      return dopplerBin < o.dopplerBin;
    }
  };

  std::map<DetectionKey, PersistentDetection> persistenceMap;

  /// @brief Quantise delay/doppler to grid for matching.
  DetectionKey quantise(double delay, double doppler) const;

  /// @brief Reset per-CPI fields prior to logging a new CPI.
  void reset_cpi_fields(uint64_t timestamp);
};

#endif
