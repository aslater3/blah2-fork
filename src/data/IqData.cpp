#include "IqData.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <stdexcept>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filewritestream.h"

// constructor
IqData::IqData(uint32_t _n)
{
  n = _n;
  buffer.resize(n);
  head = 0;
  tail = 0;
  count = 0;
}

uint32_t IqData::get_n()
{
  return n;
}

uint32_t IqData::get_length()
{
  return count;
}

uint64_t IqData::get_overflow_count() const
{
  return overflowCount;
}

void IqData::lock()
{
  mutex_lock.lock();
}

void IqData::unlock()
{
  mutex_lock.unlock();
}

std::deque<std::complex<double>> IqData::get_data()
{
  std::deque<std::complex<double>> result;
  result.resize(count);
  // Copy from ring buffer in order (head → tail)
  if (count == 0)
  {
    return result;
  }
  if (head + count <= n)
  {
    // contiguous region
    std::copy(buffer.begin() + head, buffer.begin() + head + count, result.begin());
  }
  else
  {
    // wraps around
    uint32_t firstPart = n - head;
    std::copy(buffer.begin() + head, buffer.end(), result.begin());
    std::copy(buffer.begin(), buffer.begin() + (count - firstPart), result.begin() + firstPart);
  }
  return result;
}

void IqData::push_back(std::complex<double> sample)
{
  buffer[tail] = sample;
  tail = (tail + 1) % n;

  if (count < n)
  {
    count++;
  }
  else
  {
    // Buffer was full — oldest sample overwritten, advance head
    head = tail;
    overflowCount++;
    if (overflowCount == 1 || (overflowCount % 1000000) == 0)
    {
      std::cerr << "[IqData] Buffer overflow: " << overflowCount
                << " samples dropped (buffer full at " << n << " samples)." << std::endl;
    }
  }
}

std::complex<double> IqData::pop_front()
{
  if (count == 0) {
    throw std::runtime_error("Attempting to pop from an empty buffer");
  }
  std::complex<double> sample = buffer[head];
  head = (head + 1) % n;
  count--;
  return sample;
}

uint32_t IqData::read_front(std::complex<double> *dest, uint32_t requested)
{
  uint32_t toRead = std::min(requested, count);
  if (toRead == 0)
  {
    return 0;
  }

  if (head + toRead <= n)
  {
    // contiguous — single memcpy
    std::memcpy(dest, buffer.data() + head, toRead * sizeof(std::complex<double>));
  }
  else
  {
    // wraps around — two memcpys
    uint32_t firstPart = n - head;
    std::memcpy(dest, buffer.data() + head, firstPart * sizeof(std::complex<double>));
    std::memcpy(dest + firstPart, buffer.data(), (toRead - firstPart) * sizeof(std::complex<double>));
  }

  head = (head + toRead) % n;
  count -= toRead;
  return toRead;
}

void IqData::print()
{
  std::cout << count << std::endl;
  // Print without modifying the buffer
  for (uint32_t i = 0; i < count; i++)
  {
    std::cout << buffer[(head + i) % n] << std::endl;
  }
}

void IqData::clear()
{
  head = 0;
  tail = 0;
  count = 0;
}

void IqData::update_spectrum(std::vector<std::complex<double>> _spectrum)
{
  spectrum = _spectrum;
}

void IqData::update_frequency(std::vector<double> _frequency)
{
  frequency = _frequency;
}

std::string IqData::to_json(uint64_t timestamp)
{
  rapidjson::Document document;
  document.SetObject();
  rapidjson::Document::AllocatorType &allocator = document.GetAllocator();

  // store frequency array
  rapidjson::Value arrayFrequency(rapidjson::kArrayType);
  for (size_t i = 0; i < frequency.size(); i++)
  {
    arrayFrequency.PushBack(frequency[i], allocator);
  }

  // store spectrum array
  rapidjson::Value arraySpectrum(rapidjson::kArrayType);
  for (size_t i = 0; i < spectrum.size(); i++)
  {
    arraySpectrum.PushBack(10 * std::log10(std::abs(spectrum[i])), allocator);
  }

  document.AddMember("timestamp", timestamp, allocator);
  document.AddMember("min", min, allocator);
  document.AddMember("max", max, allocator);
  document.AddMember("mean", mean, allocator);
  document.AddMember("frequency", arrayFrequency, allocator);
  document.AddMember("spectrum", arraySpectrum, allocator);

  rapidjson::StringBuffer strbuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
  writer.SetMaxDecimalPlaces(2);
  document.Accept(writer);

  return strbuf.GetString();
}