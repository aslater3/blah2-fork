#include "Kraken.h"

#include <iostream>
#include <complex>
#include <thread>
#include <algorithm>

 // constructor
Kraken::Kraken(std::string _type, uint32_t _fc, uint32_t _fs, 
  std::string _path, bool *_saveIq, std::vector<double> _gain, std::vector<std::string> _serials)
    : Source(_type, _fc, _fs, _path, _saveIq)
{
    // determine channel indices:
    if (!_serials.empty())
    {
        // map each requested serial to a device index
        int dev_count = rtlsdr_get_device_count();
        if (dev_count < 0) {
            throw std::runtime_error("[Kraken] Failed to get device count.");
        }

        for (const auto &req_serial : _serials)
        {
            bool found = false;
            for (int idx = 0; idx < dev_count; ++idx)
            {
                char vendor[256] = {0}, product[256] = {0}, serialBuf[256] = {0};
                int rc = rtlsdr_get_device_usb_strings(idx, vendor, product, serialBuf);
                if (rc < 0) {
                    // skip devices we can't query
                    continue;
                }
                if (req_serial == std::string(serialBuf))
                {
                    channelIndex.push_back(idx);
                    std::cout << "[Kraken] Mapped serial " << req_serial << " -> device index " << idx << "." << std::endl;
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                throw std::runtime_error("[Kraken] Requested serial " + req_serial + " not found.");
            }
        }
    }
    else
    {
        // default: use devices by enumerated index 0..N-1
        for (size_t i = 0; i < _gain.size(); i++)
        {
            channelIndex.push_back(static_cast<int>(i));
        }
    }

    // probe valid gains using the first mapped device (or device 0 fallback)
    std::vector<int> validGains;
    int nGains, status;
    rtlsdr_dev_t* probe_dev = nullptr;
    int probe_index = (channelIndex.size() > 0) ? channelIndex[0] : 0;
    status = rtlsdr_open(&probe_dev, probe_index);
    check_status(status, "Failed to open device for available gains.");
    nGains = rtlsdr_get_tuner_gains(probe_dev, nullptr);
    check_status(nGains, "Failed to get number of gains.");
    std::unique_ptr<int[]> _validGains(new int[nGains]);
    status = rtlsdr_get_tuner_gains(probe_dev, _validGains.get());
    check_status(status, "Failed to get tuner gains.");
    validGains.assign(_validGains.get(), _validGains.get() + nGains);
    status = rtlsdr_close(probe_dev);
    check_status(status, "Failed to close device for available gains.");

    // convert and validate gains (store only adjusted values)
    for (size_t i = 0; i < _gain.size(); i++)
    {
        int requested = static_cast<int>(_gain[i] * 10);
        auto it = std::lower_bound(validGains.begin(), validGains.end(), requested);
        int finalGain;
        if (it != validGains.end()) {
            finalGain = *it;
        } else {
            finalGain = validGains.back();
        }
        gain.push_back(finalGain);
        std::cout << "[Kraken] Gain update on channel " << i << " from " << requested << " to " << finalGain << "." << std::endl;
    }
}

void Kraken::start()
{
    int status;
    for (size_t i = 0; i < channelIndex.size(); i++) 
    {
        // fetch USB strings for logging (vendor/product/serial) where available
        char vendor[256] = {0}, product[256] = {0}, serialBuf[256] = {0};
        int rc = rtlsdr_get_device_usb_strings(channelIndex[i], vendor, product, serialBuf);
        if (rc == 0) {
            std::cout << "[Kraken] Setting up channel " << i << " (device index " << channelIndex[i] 
                      << ", serial=" << serialBuf << ", vendor=" << vendor << ", product=" << product << ")." << std::endl;
        } else {
            std::cout << "[Kraken] Setting up channel " << i << " (device index " << channelIndex[i] 
                      << "). Device USB strings unavailable." << std::endl;
        }

        status = rtlsdr_open(&devs[i], channelIndex[i]);
        check_status(status, "Failed to open device.");

        status = rtlsdr_set_center_freq(devs[i], fc);
        check_status(status, "Failed to set center frequency.");
        status = rtlsdr_set_sample_rate(devs[i], fs);
        check_status(status, "Failed to set sample rate.");
        status = rtlsdr_set_dithering(devs[i], 0); // disable dither
        check_status(status, "Failed to disable dithering.");
        status = rtlsdr_set_tuner_gain_mode(devs[i], 1); // disable AGC
        check_status(status, "Failed to disable AGC.");
        status = rtlsdr_set_tuner_gain(devs[i], gain[i]);
        check_status(status, "Failed to set gain.");
        status = rtlsdr_reset_buffer(devs[i]);
        check_status(status, "Failed to reset buffer.");
    }
}

void Kraken::stop()
{
    int status;
    for (size_t i = 0; i < channelIndex.size(); i++) 
    {
        status = rtlsdr_cancel_async(devs[i]);
        check_status(status, "Failed to stop async read.");
    }
}

void Kraken::process(IqData *buffer1, IqData *buffer2)
{
    std::vector<std::thread> threads;
    // map buffers: channel 0 -> buffer1, channel 1 -> buffer2 (if present)
    std::vector<IqData*> bufs;
    bufs.push_back(buffer1);
    if (channelIndex.size() > 1)
        bufs.push_back(buffer2);

    size_t nThreads = std::min(channelIndex.size(), bufs.size());
    for (size_t i = 0; i < nThreads; ++i)
    {
        threads.emplace_back(rtlsdr_read_async, devs[i], callback, bufs[i], 0, 16 * 16384);
    }

    // join threads
    for (auto& thread : threads) {
        thread.join();
    }
}

void Kraken::callback(unsigned char *buf, uint32_t len, void *ctx) 
{
    IqData* buffer_blah2 = (IqData*)ctx;
    int8_t* buffer_kraken = (int8_t*)buf;

    buffer_blah2->lock();

    for (size_t i = 0; i < len; i += 2) {
        double iqi = static_cast<double>(buffer_kraken[i]);
        double iqq = static_cast<double>(buffer_kraken[i + 1]);

        buffer_blah2->push_back({iqi, iqq});
    }

    buffer_blah2->unlock();
}

void Kraken::replay(IqData *buffer1, IqData *buffer2, std::string _file, bool _loop)
{
    // todo
}

void Kraken::check_status(int status, std::string message)
{
  if (status < 0)
  {
    throw std::runtime_error("[Kraken] " + message);
  }
}
