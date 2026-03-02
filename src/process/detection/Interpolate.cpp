#include "Interpolate.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <stdint.h>
#include <algorithm>

// constructor
Interpolate::Interpolate(bool _doDelay, bool _doDoppler)
{
  // input
  doDelay = _doDelay;
  doDoppler = _doDoppler;
}

Interpolate::~Interpolate()
{
}

std::unique_ptr<Detection> Interpolate::process(Detection *x, Map<std::complex<double>> *y)
{ 
  // store detections temporarily
  std::vector<double> delay, doppler, snr;
  delay = x->get_delay();
  doppler = x->get_doppler();
  snr = x->get_snr();

  // interpolate data
  double intDelay, intDoppler, intSnrDelay, intSnrDoppler, intSnr[3];
  std::vector<double> delay2, doppler2, snr2;
  std::deque<int> indexDelay = y->delay;
  std::deque<double> indexDoppler = y->doppler;

  // loop over every detection
  for (size_t i = 0; i < snr.size(); i++)
  {
    // initialise interpolated values for bool flags
    intDelay = delay[i];
    intDoppler = doppler[i];
    intSnrDelay = snr[i];
    intSnrDoppler = snr[i];

    // interpolate in delay
    if (doDelay)
    {
      // skip interpolation on boundary but keep the detection
      int delayIdx = static_cast<int>(delay[i]) - indexDelay[0];
      if (delayIdx > 0 && delayIdx < static_cast<int>(indexDelay.size()) - 1)
      {
        uint32_t dopplerBin = y->doppler_hz_to_bin(doppler[i]);
        intSnr[0] = 10.0*std::log10(std::abs(y->data[dopplerBin][delayIdx-1]) + 1e-30) - y->noisePower;
        intSnr[1] = 10.0*std::log10(std::abs(y->data[dopplerBin][delayIdx]) + 1e-30) - y->noisePower;
        intSnr[2] = 10.0*std::log10(std::abs(y->data[dopplerBin][delayIdx+1]) + 1e-30) - y->noisePower;
        // check detection has peak SNR of neighbours
        if (intSnr[1] >= intSnr[0] && intSnr[1] >= intSnr[2])
        {
          double denom = intSnr[0] - (2*intSnr[1]) + intSnr[2];
          if (std::abs(denom) > 1e-10)
          {
            double fracDelay = (intSnr[0]-intSnr[2])/(2*denom);
            intSnrDelay = intSnr[1] - (((intSnr[0]-intSnr[2])*fracDelay)/4);
            intDelay = delay[i] + fracDelay;
          }
        }
      }
    }
    // interpolate in Doppler
    if (doDoppler)
    {
      uint32_t dopplerBin = y->doppler_hz_to_bin(doppler[i]);
      int delayIdx = static_cast<int>(delay[i]) - indexDelay[0];
      // skip interpolation on boundary but keep the detection
      if (dopplerBin > 0 && dopplerBin < y->get_nRows() - 1 &&
          delayIdx >= 0 && delayIdx < static_cast<int>(indexDelay.size()))
      {
        intSnr[0] = 10.0*std::log10(std::abs(y->data[dopplerBin-1][delayIdx]) + 1e-30) - y->noisePower;
        intSnr[1] = 10.0*std::log10(std::abs(y->data[dopplerBin][delayIdx]) + 1e-30) - y->noisePower;
        intSnr[2] = 10.0*std::log10(std::abs(y->data[dopplerBin+1][delayIdx]) + 1e-30) - y->noisePower;
        // check detection has peak SNR of neighbours
        if (intSnr[1] >= intSnr[0] && intSnr[1] >= intSnr[2])
        {
          double denom = intSnr[0] - (2*intSnr[1]) + intSnr[2];
          if (std::abs(denom) > 1e-10)
          {
            double fracDoppler = (intSnr[0]-intSnr[2])/(2*denom);
            intSnrDoppler = intSnr[1] - (((intSnr[0]-intSnr[2])*fracDoppler)/4);
            intDoppler = doppler[i] + ((indexDoppler[1]-indexDoppler[0])*fracDoppler);
          }
        }
      }
    }
    // store detection (interpolated or not — never silently drop)
    delay2.push_back(intDelay);
    doppler2.push_back(intDoppler);
    snr2.push_back(std::max(std::max(intSnrDelay, intSnrDoppler), snr[i]));
  }

  // create detection
  return std::make_unique<Detection>(delay2, doppler2, snr2);
}
