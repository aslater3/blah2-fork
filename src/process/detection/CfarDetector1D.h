/// @file CfarDetector1D.h
/// @class CfarDetector1D
/// @brief A class to implement a 1D CFAR detector.
/// @details Converts an AmbiguityMap to DetectionData. 1D CFAR operates across delay, to minimise detections from the zero-Doppler line.
/// @author 30hours
/// @todo Actually implement the min delay and Doppler.

#ifndef CFARDETECTOR1D_H
#define CFARDETECTOR1D_H

#include "data/Map.h"
#include "data/Detection.h"
#include <stdint.h>
#include <complex>
#include <memory>
#include <string>
#include <fstream>
#include <vector>

class CfarDetector1D
{
private:
  /// @brief Probability of false alarm, numeric in [0,1]
  double pfa;

  /// @brief Number of single-sided guard cells.
  int8_t nGuard;

  /// @brief Number of single-sided training cells.
  int8_t nTrain;

  /// @brief Minimum delay to process detections (bins).
  int8_t minDelay;

  /// @brief Minimum absolute Doppler to process detections (Hz).
  double minDoppler;

  /// @brief Pointer to detection data to store result.
  Detection *detection;

  /// @brief Per-row CFAR diagnostics from the last processed CPI.
  std::vector<double> rowDopplerHz;
  std::vector<double> rowNoiseFloorDb;
  std::vector<double> rowThresholdDb;
  std::vector<uint32_t> rowDetectionCount;

  /// @brief Optional debug mode writing per-cell threshold/test-statistic values.
  bool debugEnabled;
  std::ofstream debugFile;
  bool rowMetricsEnabled;

public:
  /// @brief Constructor.
  /// @param pfa Probability of false alarm, numeric in [0,1].
  /// @param nGuard Number of single-sided guard cells.
  /// @param nTrain Number of single-sided training cells.
  /// @param minDelay Minimum delay to process detections (bins).
  /// @param minDoppler Minimum absolute Doppler to process detections (Hz).
  /// @return The object.
  CfarDetector1D(double pfa, int8_t nGuard, int8_t nTrain, int8_t minDelay, double minDoppler);

  /// @brief Destructor.
  /// @return Void.
  ~CfarDetector1D();

  /// @brief Implement the 1D CFAR detector.
  /// @param x Ambiguity map data of IQ samples.
  /// @param timestamp Current CPI timestamp (POSIX ms), used by debug logging.
  /// @return Detections from the 1D CFAR detector.
  std::unique_ptr<Detection> process(Map<std::complex<double>> *x, uint64_t timestamp = 0);

  /// @brief Enable/disable CFAR debug file logging.
  /// @param enable True to enable per-cell debug output.
  /// @param path Output JSONL file path.
  void set_debug_logging(bool enable, const std::string &path);

  /// @brief Last per-row Doppler values (Hz) from the most recent process() call.
  const std::vector<double> &get_row_doppler() const;

  /// @brief Last per-row noise floor values (dB).
  const std::vector<double> &get_row_noise_floor() const;

  /// @brief Last per-row mean CFAR threshold values (dB).
  const std::vector<double> &get_row_threshold() const;

  /// @brief Last per-row detection counts.
  const std::vector<uint32_t> &get_row_detection_count() const;

  /// @brief Enable/disable per-row CFAR metric collection.
  void set_row_metrics_enabled(bool enable);
};

#endif
