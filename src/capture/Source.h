/// @file Source.h
/// @class Source
/// @brief An abstract class for capture sources.
/// @author 30hours

#ifndef SOURCE_H
#define SOURCE_H

#include <string>
#include <stdint.h>
#include <fstream>
#include <atomic>
#include "data/IqData.h"

class Source
{
protected:

  /// @brief The capture device type.
  std::string type;

  /// @brief Center frequency (Hz).
  uint32_t fc;

  /// @brief Sampling frequency (Hz).
  uint32_t fs;

  /// @brief Absolute path to IQ save location.
  std::string path;

  /// @brief True if IQ data to be saved.
  bool *saveIq;

  /// @brief File stream to save IQ data.
  std::ofstream saveIqFile;

public:

  Source();

  /// @brief Constructor.
  /// @param type The capture device type.
  /// @param fs Sampling frequency (Hz).
  /// @param fc Center frequency (Hz).
  /// @param path Absolute path to IQ save location.
  /// @return The object.
  Source(std::string type, uint32_t fc, uint32_t fs, 
    std::string path, bool *saveIq);

  /// @brief Implement the capture process.
  /// @param buffer1 Buffer for reference samples.
  /// @param buffer2 Buffer for surveillance samples.
  /// @return Void.
  virtual void process(IqData *buffer1, IqData *buffer2) = 0;

  /// @brief Call methods to start capture.
  /// @return Void.
  virtual void start() = 0;

  /// @brief Call methods to gracefully stop capture.
  /// @return Void.
  virtual void stop() = 0;

  /// @brief Implement replay function on RSPduo.
  /// @param buffer1 Pointer to reference buffer.
  /// @param buffer2 Pointer to surveillance buffer.
  /// @param file Path to file to replay data from.
  /// @param loop True if samples should loop at EOF.
  /// @return Void.
  virtual void replay(IqData *buffer1, IqData *buffer2, 
    std::string file, bool loop) = 0;

  /// @brief Return the latest sync-calibration metrics if supported by device.
  /// @param offsetSamples Latest measured sample offset.
  /// @param snrDb Latest measured sync correlation peak SNR in dB.
  /// @return True if sync metrics are available.
  virtual bool get_sync_metrics(int64_t &offsetSamples, double &snrDb) const;

  /// @brief Return latest estimated sample-drop counters per channel if available.
  /// @param ch0 Estimated drops for channel 0.
  /// @param ch1 Estimated drops for channel 1.
  /// @return True if sample-drop metrics are available.
  virtual bool get_sample_drop_metrics(uint64_t &ch0, uint64_t &ch1) const;

  /// @brief Open a new file to record IQ.
  /// @details First creates a new file from current timestamp.
  /// Files are of format <path>.<type>.iq.
  /// @return String of full path to file.
  std::string open_file();

  /// @brief Close IQ file gracefully.
  /// @return Void.
  void close_file();

  /// @brief Graceful handler for SIGTERM.
  /// @return Void.
  void kill();

};

#endif
