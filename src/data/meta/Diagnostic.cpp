#include "Diagnostic.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <set>
#include <algorithm>

Diagnostic::Diagnostic(const std::string &_savePath, double _delayTolerance,
                       double _dopplerTolerance, uint32_t _persistenceThreshold)
    : savePath(_savePath), delayTolerance(_delayTolerance),
      dopplerTolerance(_dopplerTolerance), persistenceThreshold(_persistenceThreshold)
{
  reset_cpi_fields(0);

  std::string logPath = savePath + ".diagnostic.jsonl";
  logFile.open(logPath, std::ios::app);
  if (!logFile.is_open())
  {
    std::cerr << "[Diagnostic] Warning: could not open " << logPath << std::endl;
  }

  std::string zeroPath = savePath + ".zero_doppler.jsonl";
  zeroDopplerFile.open(zeroPath, std::ios::app);
  if (!zeroDopplerFile.is_open())
  {
    std::cerr << "[Diagnostic] Warning: could not open " << zeroPath << std::endl;
  }
}

Diagnostic::~Diagnostic()
{
  if (logFile.is_open())
  {
    logFile.close();
  }
  if (zeroDopplerFile.is_open())
  {
    zeroDopplerFile.close();
  }
}

Diagnostic::DetectionKey Diagnostic::quantise(double delay, double doppler) const
{
  double delayStep = std::max(delayTolerance, 1e-3);
  double dopplerStep = std::max(dopplerTolerance, 1e-3);
  return {static_cast<int>(std::round(delay / delayStep)),
          static_cast<int>(std::round(doppler / dopplerStep))};
}

void Diagnostic::reset_cpi_fields(uint64_t timestamp)
{
  current.timestamp = timestamp;
  current.noisePower_dB = -300.0;
  current.maxPower_dB = 0.0;
  current.nDetections_raw = 0;
  current.nDetections_centroid = 0;
  current.nDetections_final = 0;
  current.clutterPower_before_dB = -300.0;
  current.clutterPower_after_dB = -300.0;
  current.clutterSuppression_dB = 0.0;
  current.syncOffset_samples = 0;
  current.syncSNR_dB = 0.0;
  current.syncValid = false;
  current.sampleDrops_ch0 = 0;
  current.sampleDrops_ch1 = 0;
  current.cfarRows.clear();
  current.detectionStates.clear();
  current.nRecurring = 0;
  current.recurringDetections.clear();
}

void Diagnostic::set_clutter_power(double before, double after)
{
  current.clutterPower_before_dB = 10.0 * std::log10(before + 1e-30);
  current.clutterPower_after_dB = 10.0 * std::log10(after + 1e-30);
  current.clutterSuppression_dB = current.clutterPower_before_dB - current.clutterPower_after_dB;
}

void Diagnostic::set_detection_counts(uint32_t raw, uint32_t centroid, uint32_t final_count)
{
  current.nDetections_raw = raw;
  current.nDetections_centroid = centroid;
  current.nDetections_final = final_count;
}

void Diagnostic::set_map_metrics(double noisePower, double maxPower)
{
  current.noisePower_dB = noisePower;
  current.maxPower_dB = maxPower;
}

void Diagnostic::set_sync_metrics(int64_t offsetSamples, double snrDb, bool valid)
{
  current.syncOffset_samples = offsetSamples;
  current.syncSNR_dB = snrDb;
  current.syncValid = valid;
}

void Diagnostic::set_sample_drops(uint64_t ch0, uint64_t ch1)
{
  current.sampleDrops_ch0 = ch0;
  current.sampleDrops_ch1 = ch1;
}

void Diagnostic::set_cfar_rows(const std::vector<double> &doppler,
                               const std::vector<double> &noisePower,
                               const std::vector<double> &threshold,
                               const std::vector<uint32_t> &nDetections)
{
  current.cfarRows.clear();
  size_t n = std::min(doppler.size(), noisePower.size());
  n = std::min(n, threshold.size());
  n = std::min(n, nDetections.size());
  current.cfarRows.reserve(n);
  for (size_t i = 0; i < n; i++)
  {
    CfarRowMetric row;
    row.doppler = doppler[i];
    row.noisePower_dB = noisePower[i];
    row.threshold_dB = threshold[i];
    row.nDetections = nDetections[i];
    current.cfarRows.push_back(row);
  }
}

void Diagnostic::update_persistence(const std::vector<double> &delay,
                                    const std::vector<double> &doppler,
                                    const std::vector<double> &snr,
                                    uint64_t timestamp)
{
  current.detectionStates.clear();

  std::map<DetectionKey, PersistentDetection> updated;
  std::set<DetectionKey> usedOld;

  size_t n = std::min(delay.size(), doppler.size());
  n = std::min(n, snr.size());
  current.detectionStates.reserve(n);

  for (size_t i = 0; i < n; i++)
  {
    const double d = delay[i];
    const double f = doppler[i];
    const double s = snr[i];
    DetectionKey key = quantise(d, f);

    DetectionKey bestKey = key;
    bool found = false;
    double bestDistance = 1e30;

    for (auto &kv : persistenceMap)
    {
      if (usedOld.find(kv.first) != usedOld.end())
      {
        continue;
      }
      double delayErr = std::abs(kv.second.delay - d);
      double dopplerErr = std::abs(kv.second.doppler - f);
      if (delayErr <= delayTolerance && dopplerErr <= dopplerTolerance)
      {
        double score = delayErr + dopplerErr;
        if (score < bestDistance)
        {
          bestDistance = score;
          bestKey = kv.first;
          found = true;
        }
      }
    }

    PersistentDetection pd;
    DetectionState state;
    state.delay = d;
    state.doppler = f;
    state.snr = s;

    if (!found)
    {
      pd.delay = d;
      pd.doppler = f;
      pd.snr = s;
      pd.persistence = 1;
      pd.firstSeen = timestamp;
      pd.lastSeen = timestamp;
      state.persistence = 1;
      state.status = "NEW";
    }
    else
    {
      pd = persistenceMap[bestKey];
      pd.delay = d;
      pd.doppler = f;
      pd.snr = s;
      pd.persistence += 1;
      pd.lastSeen = timestamp;
      state.persistence = pd.persistence;
      if (bestKey.delayBin == key.delayBin && bestKey.dopplerBin == key.dopplerBin)
      {
        state.status = "RECURRING";
      }
      else
      {
        state.status = "MOVING";
      }
      usedOld.insert(bestKey);
    }

    auto itExisting = updated.find(key);
    if (itExisting == updated.end() || pd.persistence >= itExisting->second.persistence)
    {
      updated[key] = pd;
    }

    current.detectionStates.push_back(state);
  }

  persistenceMap.swap(updated);

  current.recurringDetections.clear();
  current.nRecurring = 0;
  for (auto &kv : persistenceMap)
  {
    if (kv.second.persistence >= persistenceThreshold)
    {
      current.recurringDetections.push_back(kv.second);
      current.nRecurring++;
    }
  }
}

void Diagnostic::log_cpi(uint64_t timestamp)
{
  current.timestamp = timestamp;

  if (!logFile.is_open())
  {
    return;
  }

  // write JSONL (one JSON object per line)
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "{";
  ss << "\"timestamp\":" << timestamp;
  ss << ",\"noisePower_dB\":" << current.noisePower_dB;
  ss << ",\"maxPower_dB\":" << current.maxPower_dB;
  ss << ",\"nDetections_raw\":" << current.nDetections_raw;
  ss << ",\"nDetections_centroid\":" << current.nDetections_centroid;
  ss << ",\"nDetections_final\":" << current.nDetections_final;
  ss << ",\"clutterPower_before_dB\":" << current.clutterPower_before_dB;
  ss << ",\"clutterPower_after_dB\":" << current.clutterPower_after_dB;
  ss << ",\"clutterSuppression_dB\":" << current.clutterSuppression_dB;
  ss << ",\"syncOffset_samples\":" << current.syncOffset_samples;
  ss << ",\"syncSNR_dB\":" << current.syncSNR_dB;
  ss << ",\"syncValid\":" << (current.syncValid ? "true" : "false");
  ss << ",\"sampleDrops_ch0\":" << current.sampleDrops_ch0;
  ss << ",\"sampleDrops_ch1\":" << current.sampleDrops_ch1;
  ss << ",\"nRecurring\":" << current.nRecurring;

  if (!current.cfarRows.empty())
  {
    ss << ",\"cfarRows\":[";
    for (size_t i = 0; i < current.cfarRows.size(); i++)
    {
      const auto &row = current.cfarRows[i];
      if (i > 0) ss << ",";
      ss << "{\"doppler\":" << row.doppler;
      ss << ",\"noisePower_dB\":" << row.noisePower_dB;
      ss << ",\"threshold_dB\":" << row.threshold_dB;
      ss << ",\"nDetections\":" << row.nDetections;
      ss << "}";
    }
    ss << "]";
  }

  if (!current.detectionStates.empty())
  {
    ss << ",\"detectionStates\":[";
    for (size_t i = 0; i < current.detectionStates.size(); i++)
    {
      const auto &state = current.detectionStates[i];
      if (i > 0) ss << ",";
      ss << "{\"delay\":" << state.delay;
      ss << ",\"doppler\":" << state.doppler;
      ss << ",\"snr\":" << state.snr;
      ss << ",\"persistence\":" << state.persistence;
      ss << ",\"status\":\"" << state.status << "\"";
      ss << "}";
    }
    ss << "]";
  }

  if (!current.recurringDetections.empty())
  {
    ss << ",\"recurring\":[";
    for (size_t i = 0; i < current.recurringDetections.size(); i++)
    {
      auto &rd = current.recurringDetections[i];
      if (i > 0) ss << ",";
      ss << "{\"delay\":" << rd.delay;
      ss << ",\"doppler\":" << rd.doppler;
      ss << ",\"snr\":" << rd.snr;
      ss << ",\"persistence\":" << rd.persistence;
      ss << ",\"firstSeen\":" << rd.firstSeen;
      ss << ",\"lastSeen\":" << rd.lastSeen;
      ss << "}";
    }
    ss << "]";
  }

  ss << "}";

  logFile << ss.str() << std::endl;
  logFile.flush();
}

void Diagnostic::log_zero_doppler_row(uint64_t timestamp,
                                      const std::vector<double> &before_dB,
                                      const std::vector<double> &after_dB)
{
  if (!zeroDopplerFile.is_open())
  {
    return;
  }

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "{";
  ss << "\"timestamp\":" << timestamp;
  ss << ",\"before_dB\":[";
  for (size_t i = 0; i < before_dB.size(); i++)
  {
    if (i > 0) ss << ",";
    ss << before_dB[i];
  }
  ss << "]";
  ss << ",\"after_dB\":[";
  for (size_t i = 0; i < after_dB.size(); i++)
  {
    if (i > 0) ss << ",";
    ss << after_dB[i];
  }
  ss << "]";
  ss << "}";

  zeroDopplerFile << ss.str() << std::endl;
  zeroDopplerFile.flush();
}

std::string Diagnostic::recurring_to_json() const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "{\"nRecurring\":" << current.nRecurring;
  ss << ",\"persistenceThreshold\":" << persistenceThreshold;
  ss << ",\"detections\":[";
  for (size_t i = 0; i < current.recurringDetections.size(); i++)
  {
    auto &rd = current.recurringDetections[i];
    if (i > 0) ss << ",";
    ss << "{\"delay\":" << rd.delay;
    ss << ",\"doppler\":" << rd.doppler;
    ss << ",\"snr\":" << rd.snr;
    ss << ",\"persistence\":" << rd.persistence;
    ss << "}";
  }
  ss << "]}";
  return ss.str();
}

CpiDiagnostic Diagnostic::get_current() const
{
  return current;
}
