#include "CfarDetector1D.h"
#include "data/Map.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>

// constructor
CfarDetector1D::CfarDetector1D(double _pfa, int8_t _nGuard, int8_t _nTrain, int8_t _minDelay, double _minDoppler)
{
  // input
  pfa = _pfa;
  nGuard = _nGuard;
  nTrain = _nTrain;
  minDelay = _minDelay;
  minDoppler = _minDoppler;
  debugEnabled = false;
}

CfarDetector1D::~CfarDetector1D()
{
  if (debugFile.is_open())
  {
    debugFile.close();
  }
}

void CfarDetector1D::set_debug_logging(bool enable, const std::string &path)
{
  debugEnabled = enable;
  if (debugFile.is_open())
  {
    debugFile.close();
  }
  if (debugEnabled)
  {
    debugFile.open(path, std::ios::app);
    if (!debugFile.is_open())
    {
      std::cerr << "[CFAR] Warning: could not open debug file " << path << std::endl;
      debugEnabled = false;
    }
  }
}

const std::vector<double> &CfarDetector1D::get_row_doppler() const
{
  return rowDopplerHz;
}

const std::vector<double> &CfarDetector1D::get_row_noise_floor() const
{
  return rowNoiseFloorDb;
}

const std::vector<double> &CfarDetector1D::get_row_threshold() const
{
  return rowThresholdDb;
}

const std::vector<uint32_t> &CfarDetector1D::get_row_detection_count() const
{
  return rowDetectionCount;
}

std::unique_ptr<Detection> CfarDetector1D::process(Map<std::complex<double>> *x, uint64_t timestamp)
{ 
  int32_t nDelayBins = x->get_nCols();
  int32_t nDopplerBins = x->get_nRows();

  std::vector<std::complex<double>> mapRow;
  std::vector<double> mapRowSquare, mapRowSnr;

  // store detections temporarily
  std::vector<double> delay;
  std::vector<double> doppler;
  std::vector<double> snr;

  rowDopplerHz.clear();
  rowNoiseFloorDb.clear();
  rowThresholdDb.clear();
  rowDetectionCount.clear();

  // loop over every cell
  for (int i = 0; i < nDopplerBins; i++)
  { 
    // skip if less than min Doppler
    if (std::abs(x->doppler[i]) < minDoppler)
    {
      continue;
    } 
    mapRow = x->get_row(i);
    double rowPowerSum = 0.0;
    for (int j = 0; j < nDelayBins; j++)
    {
      double mag = std::abs(mapRow[j]);
      double power = mag * mag;
      mapRowSquare.push_back(power);
       rowPowerSum += power;
      // SNR in dB: 10*log10(|z|^2) - noisePower = 20*log10(|z|) - noisePower
      // Use consistent 10*log10(|z|) to match noisePower scale from set_metrics()
      mapRowSnr.push_back(10.0 * std::log10(mag + 1e-30) - x->noisePower);
    }

    uint32_t rowDetections = 0;
    double thresholdSumDb = 0.0;
    uint32_t thresholdCount = 0;
    std::vector<double> rowThresholdDebug;
    std::vector<uint8_t> rowDetectionDebug;
    if (debugEnabled)
    {
      rowThresholdDebug.assign(static_cast<size_t>(nDelayBins), -300.0);
      rowDetectionDebug.assign(static_cast<size_t>(nDelayBins), 0);
    }

    for (int j = 0; j < nDelayBins; j++)
    {
      // skip if less than min delay
      if (x->delay[j] < minDelay)
      {
        continue;
      } 
      // get train cell indices
      std::vector<int> iTrain;
      for (int k = j-nGuard-nTrain; k < j-nGuard; k++)
      {
        if (k >= 0 && k < nDelayBins)
        {
          iTrain.push_back(k);
        }
      }
      for (int k = j+nGuard+1; k < j+nGuard+nTrain+1; k++)
      {
        if (k >= 0 && k < nDelayBins)
        {
          iTrain.push_back(k);
        }
      }

      // compute threshold
      int nCells = iTrain.size();
      if (nCells == 0)
      {
        continue;
      }
      double alpha = nCells * (pow(pfa, -1.0 / nCells) - 1);
      double trainNoise = 0.0;
      for (int k = 0; k < nCells; k++)
      {
        trainNoise += mapRowSquare[iTrain[k]];
      }
      trainNoise /= nCells;
      double threshold = alpha * trainNoise;
      double thresholdDb = 10.0 * std::log10(std::sqrt(threshold) + 1e-30) - x->noisePower;
      thresholdSumDb += thresholdDb;
      thresholdCount++;
      if (debugEnabled)
      {
        rowThresholdDebug[static_cast<size_t>(j)] = thresholdDb;
      }

      // detection if over threshold
      if (mapRowSquare[j] > threshold)
      {
        delay.push_back(j + x->delay[0]);
        doppler.push_back(x->doppler[i]);
        snr.push_back(mapRowSnr[j]);
        rowDetections++;
        if (debugEnabled)
        {
          rowDetectionDebug[static_cast<size_t>(j)] = 1;
        }
      }
      iTrain.clear();
    }

    double rowNoiseDb = 10.0 * std::log10(std::sqrt(rowPowerSum / static_cast<double>(nDelayBins)) + 1e-30);
    double rowThresholdMeanDb = (thresholdCount > 0)
      ? (thresholdSumDb / static_cast<double>(thresholdCount))
      : -300.0;
    rowDopplerHz.push_back(x->doppler[i]);
    rowNoiseFloorDb.push_back(rowNoiseDb);
    rowThresholdDb.push_back(rowThresholdMeanDb);
    rowDetectionCount.push_back(rowDetections);

    if (debugEnabled && debugFile.is_open())
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << "{";
      ss << "\"timestamp\":" << timestamp;
      ss << ",\"rowIndex\":" << i;
      ss << ",\"doppler\":" << x->doppler[i];
      ss << ",\"noisePower_dB\":" << rowNoiseDb;
      ss << ",\"threshold_dB\":[";
      for (int j = 0; j < nDelayBins; j++)
      {
        if (j > 0) ss << ",";
        ss << rowThresholdDebug[static_cast<size_t>(j)];
      }
      ss << "]";
      ss << ",\"testStatistic_dB\":[";
      for (int j = 0; j < nDelayBins; j++)
      {
        if (j > 0) ss << ",";
        ss << mapRowSnr[static_cast<size_t>(j)];
      }
      ss << "]";
      ss << ",\"isDetection\":[";
      for (int j = 0; j < nDelayBins; j++)
      {
        if (j > 0) ss << ",";
        ss << static_cast<int>(rowDetectionDebug[static_cast<size_t>(j)]);
      }
      ss << "]";
      ss << "}";
      debugFile << ss.str() << std::endl;
      debugFile.flush();
    }

    mapRowSquare.clear();
    mapRowSnr.clear();
  }

  // create detection
  return std::make_unique<Detection>(delay, doppler, snr);
}
