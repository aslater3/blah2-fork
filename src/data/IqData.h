/// @file IqData.h
/// @class IqData
/// @brief A class to store IQ data.
/// @details Implements a contiguous circular buffer to store IQ samples.
///          Replaces the previous deque-based implementation for better
///          cache performance and reduced lock contention.
/// @author 30hours

#ifndef IQDATA_H
#define IQDATA_H

#include <stdint.h>
#include <deque>
#include <vector>
#include <complex>
#include <mutex>

class IqData
{
private:
  /// @brief Maximum number of samples (capacity).
  uint32_t n;

  /// @brief Contiguous circular buffer storage.
  std::vector<std::complex<double>> buffer;

  /// @brief Read position (index of oldest sample).
  uint32_t head;

  /// @brief Write position (index of next write slot).
  uint32_t tail;

  /// @brief Current number of samples in the buffer.
  uint32_t count;

  /// @brief Counter for buffer overflow events.
  uint64_t overflowCount = 0;

  /// @brief Mutex for thread safety.
  std::mutex mutex_lock;

  /// @brief Minimum value.
  double min;

  /// @brief Maximum value.
  double max;

  /// @brief Mean value.
  double mean;

  /// @brief Spectrum vector.
  std::vector<std::complex<double>> spectrum;

  /// @brief Frequency vector (Hz).
  std::vector<double> frequency;

public:
  /// @brief Constructor.
  /// @param n Number of samples (capacity).
  /// @return The object.
  IqData(uint32_t n);

  /// @brief Getter for maximum number of samples.
  /// @return Maximum number of samples.
  uint32_t get_n();

  /// @brief Getter for current data length.
  /// @return Number of samples currently in data.
  uint32_t get_length();

  /// @brief Getter for cumulative buffer overflow count.
  /// @return Number of samples dropped due to full buffer.
  uint64_t get_overflow_count() const;

  /// @brief Locker for mutex.
  /// @return Void.
  void lock();

  /// @brief Unlocker for mutex.
  /// @return Void.
  void unlock();

  /// @brief Getter for data as a deque (backward compatibility).
  /// @details Constructs a deque from the circular buffer contents.
  ///          Use sparingly — prefer direct access or read_front() in hot paths.
  /// @return IQ data as deque.
  std::deque<std::complex<double>> get_data();

  /// @brief Push a sample to the queue.
  /// @details When the buffer is full, the oldest sample is silently
  ///          overwritten and the overflow counter is incremented.
  /// @param sample A single sample.
  /// @return Void.
  void push_back(std::complex<double> sample);

  /// @brief Pop the front of the queue.
  /// @return Sample from the front of the queue.
  std::complex<double> pop_front();

  /// @brief Bulk read and remove samples from the front.
  /// @details Copies up to `requested` samples into `dest` and removes them
  ///          from the buffer. Much faster than repeated pop_front() calls.
  /// @param dest Destination array (must have room for `requested` elements).
  /// @param requested Number of samples to read.
  /// @return Actual number of samples read (may be less if buffer has fewer).
  uint32_t read_front(std::complex<double> *dest, uint32_t requested);

  /// @brief Print to stdout (debug).
  /// @return Void.
  void print();

  /// @brief Clear samples from the queue.
  /// @return Void.
  void clear();

  /// @brief Update the time differences and names.
  /// @param spectrum Spectrum vector.
  /// @return Void.
  void update_spectrum(std::vector<std::complex<double>> spectrum);

  /// @brief Update the time differences and names.
  /// @param frequency Frequency vector.
  /// @return Void.
  void update_frequency(std::vector<double> frequency);

  /// @brief Generate JSON of the signal and metadata.
  /// @param timestamp Current time (POSIX ms).
  /// @return JSON string.
  std::string to_json(uint64_t timestamp);
};

#endif