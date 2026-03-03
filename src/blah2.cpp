/// @file blah2.cpp
/// @brief A real-time radar.
/// @author 30hours

#include "capture/Capture.h"
#include "data/IqData.h"
#include "data/Map.h"
#include "data/Detection.h"
#include "data/meta/Timing.h"
#include "data/meta/Diagnostic.h"
#include "data/Track.h"
#include "process/ambiguity/Ambiguity.h"
#include "process/clutter/WienerHopf.h"
#include "process/detection/CfarDetector1D.h"
#include "process/detection/Centroid.h"
#include "process/detection/Interpolate.h"
#include "process/spectrum/SpectrumAnalyser.h"
#include "process/tracker/Tracker.h"
#include "process/utility/Socket.h"
#include "data/meta/Constants.h"

#include <ryml/ryml.hpp>
#include <ryml/ryml_std.hpp> // optional header, provided for std:: interop
#include <c4/format.hpp> // needed for the examples below
#include <sys/types.h>
#include <getopt.h>
#include <string>
#include <vector>
#include <deque>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <signal.h>
#include <atomic>
#include <memory>
#include <iostream>
#include <cmath>
#include <algorithm>

Capture *CAPTURE_POINTER = NULL;

void signal_callback_handler(int signum);
void getopt_print_help();
std::string getopt_process(int argc, char **argv);
std::string ryml_get_file(const char *filename);
uint64_t current_time_ms();
uint64_t current_time_us();
void timing_helper(std::vector<std::string>& timing_name, 
  std::vector<double>& timing_time, std::vector<uint64_t>& time_us, 
  std::string name);
double zero_lag_power(const std::deque<std::complex<double>> &x,
  const std::deque<std::complex<double>> &y);
std::vector<double> zero_doppler_row_db(Map<std::complex<double>> *map);

int main(int argc, char **argv)
{
  // input handling
  signal(SIGTERM, signal_callback_handler);
  std::string file = getopt_process(argc, argv);
  std::ifstream filePath(file);
  if (!filePath.is_open())
  {
    std::cout << "Error: Config file does not exist." << std::endl;
    exit(1);
  }

  // config handling
  std::string contents = ryml_get_file(file.c_str());
  ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr(contents));

  // setup capture
  uint32_t fs, fc;
  uint16_t port_capture;
  std::string type, path, replayFile, ip_capture;
  bool saveIq, state, loop;
  tree["capture"]["fs"] >> fs;
  tree["capture"]["fc"] >> fc;
  tree["capture"]["device"]["type"] >> type;
  tree["save"]["iq"] >> saveIq;
  tree["save"]["path"] >> path;
  tree["capture"]["replay"]["state"] >> state;
  tree["capture"]["replay"]["loop"] >> loop;
  tree["capture"]["replay"]["file"] >> replayFile;
  tree["network"]["ip"] >> ip_capture;
  tree["network"]["ports"]["api"] >> port_capture;
  Capture *capture = new Capture(type, fs, fc, path);
  CAPTURE_POINTER = capture;
  if (state)
  {
    capture->set_replay(loop, replayFile);
  }

  // create shared queue
  double tCpi, tBuffer;
  tree["process"]["data"]["cpi"] >> tCpi;
  tree["process"]["data"]["buffer"] >> tBuffer;
  IqData *buffer1 = new IqData((int) (tCpi*tBuffer*fs));
  IqData *buffer2 = new IqData((int) (tCpi*tBuffer*fs));

  // run capture
  std::thread t1([&]{capture->process(buffer1, buffer2, 
    tree["capture"]["device"], ip_capture, port_capture);
  });

  // setup process CPI
  uint32_t nSamples = fs * tCpi;
  IqData *x = new IqData(nSamples);
  IqData *y = new IqData(nSamples);
  // Pre-allocate bulk transfer buffers to avoid repeated allocation
  std::vector<std::complex<double>> bulkX(nSamples);
  std::vector<std::complex<double>> bulkY(nSamples);
  Map<std::complex<double>> *map;
  std::unique_ptr<Detection> detection;
  std::unique_ptr<Detection> detection1;
  std::unique_ptr<Detection> detection2;
  std::unique_ptr<Track> track;

  // setup fftw multithread
  if (fftw_init_threads() == 0)
  {
    std::cout << "Error in FFTW multithreading." << std::endl;
    return -1;
  }
  fftw_plan_with_nthreads(4);

  // setup socket
  sleep(5);
  uint16_t port_map, port_detection, port_timestamp, 
    port_timing, port_iqdata, port_track;
  std::string ip;
  tree["network"]["ports"]["map"] >> port_map;
  tree["network"]["ports"]["detection"] >> port_detection;
  tree["network"]["ports"]["track"] >> port_track;
  tree["network"]["ports"]["timestamp"] >> port_timestamp;
  tree["network"]["ports"]["timing"] >> port_timing;
  tree["network"]["ports"]["iqdata"] >> port_iqdata;
  tree["network"]["ip"] >> ip;

  bool streamMap = true;
  bool streamDetection = true;
  bool streamTrack = true;
  bool streamTimestamp = true;
  bool streamTiming = true;
  bool streamIqData = true;
  auto streamNode = tree["network"].find_child(c4::to_csubstr("stream"));
  if (streamNode.valid())
  {
    auto mapNode = streamNode.find_child(c4::to_csubstr("map"));
    if (mapNode.valid()) mapNode >> streamMap;
    auto detectionNode = streamNode.find_child(c4::to_csubstr("detection"));
    if (detectionNode.valid()) detectionNode >> streamDetection;
    auto trackNode = streamNode.find_child(c4::to_csubstr("track"));
    if (trackNode.valid()) trackNode >> streamTrack;
    auto timestampNode = streamNode.find_child(c4::to_csubstr("timestamp"));
    if (timestampNode.valid()) timestampNode >> streamTimestamp;
    auto timingNode = streamNode.find_child(c4::to_csubstr("timing"));
    if (timingNode.valid()) timingNode >> streamTiming;
    auto iqNode = streamNode.find_child(c4::to_csubstr("iqdata"));
    if (iqNode.valid()) iqNode >> streamIqData;
  }

  std::unique_ptr<Socket> socket_map;
  std::unique_ptr<Socket> socket_detection;
  std::unique_ptr<Socket> socket_track;
  std::unique_ptr<Socket> socket_timestamp;
  std::unique_ptr<Socket> socket_timing;
  std::unique_ptr<Socket> socket_iqdata;

  if (streamMap)
  {
    socket_map = std::make_unique<Socket>(ip, port_map);
  }
  if (streamDetection)
  {
    socket_detection = std::make_unique<Socket>(ip, port_detection);
  }
  if (streamTrack)
  {
    socket_track = std::make_unique<Socket>(ip, port_track);
  }
  if (streamTimestamp)
  {
    socket_timestamp = std::make_unique<Socket>(ip, port_timestamp);
  }
  if (streamTiming)
  {
    socket_timing = std::make_unique<Socket>(ip, port_timing);
  }
  if (streamIqData)
  {
    socket_iqdata = std::make_unique<Socket>(ip, port_iqdata);
  }

  // setup process ambiguity
  int32_t delayMin, delayMax;
  int32_t dopplerMin, dopplerMax;
  bool roundHamming = true;
  tree["process"]["ambiguity"]["delayMin"] >> delayMin;
  tree["process"]["ambiguity"]["delayMax"] >> delayMax;
  tree["process"]["ambiguity"]["dopplerMin"] >> dopplerMin;
  tree["process"]["ambiguity"]["dopplerMax"] >> dopplerMax;
  Ambiguity *ambiguity = new Ambiguity(delayMin, delayMax, 
    dopplerMin, dopplerMax, fs, nSamples, roundHamming);

  // setup process clutter
  int32_t delayMinClutter, delayMaxClutter;
  tree["process"]["clutter"]["delayMin"] >> delayMinClutter;
  tree["process"]["clutter"]["delayMax"] >> delayMaxClutter;
  WienerHopf *filter = new WienerHopf(delayMinClutter, delayMaxClutter, nSamples);

  // setup process detection
  double pfa, minDoppler;
  int8_t nGuard, nTrain;
  int8_t minDelay;
  tree["process"]["detection"]["pfa"] >> pfa;
  tree["process"]["detection"]["nGuard"] >> nGuard;
  tree["process"]["detection"]["nTrain"] >> nTrain;
  tree["process"]["detection"]["minDelay"] >> minDelay;
  tree["process"]["detection"]["minDoppler"] >> minDoppler;
  CfarDetector1D *cfarDetector1D = new CfarDetector1D(pfa, nGuard, nTrain, minDelay, minDoppler);
  bool pilotNotchEnable = true;
  auto pilotNotchNode = tree["process"]["detection"].find_child(c4::to_csubstr("pilotNotchEnable"));
  if (pilotNotchNode.valid())
  {
    pilotNotchNode >> pilotNotchEnable;
  }
  cfarDetector1D->set_pilot_notch_enabled(pilotNotchEnable);
  Interpolate *interpolate = new Interpolate(true, true);
  bool cfarDebug = false;
  auto cfarDebugNode = tree["process"]["detection"].find_child(c4::to_csubstr("cfarDebug"));
  if (cfarDebugNode.valid())
  {
    cfarDebugNode >> cfarDebug;
  }

  // setup process centroid
  // use actual CPI from ambiguity processor (accounts for integer truncation)
  uint16_t nCentroid;
  tree["process"]["detection"]["nCentroid"] >> nCentroid;
  Centroid *centroid = new Centroid(nCentroid, nCentroid, 1.0/ambiguity->get_cpi());

  // setup process tracker
  uint8_t m, n, nDelete;
  double maxAcc, rangeRes, lambda;
  std::string smooth;
  tree["process"]["tracker"]["initiate"]["M"] >> m;
  tree["process"]["tracker"]["initiate"]["N"] >> n;
  tree["process"]["tracker"]["delete"] >> nDelete;
  tree["process"]["tracker"]["initiate"]["maxAcc"] >> maxAcc;
  rangeRes = (double)Constants::c/fs;
  lambda = (double)Constants::c/fc;
  Tracker *tracker = new Tracker(m, n, nDelete, ambiguity->get_cpi(), maxAcc, rangeRes, lambda);

  // setup process spectrum analyser
  double spectrumBandwidth = 2000;
  SpectrumAnalyser *spectrumAnalyser = new SpectrumAnalyser(nSamples, spectrumBandwidth);

  // process options
  bool isClutter, isDetection, isTracker;
  tree["process"]["clutter"]["enable"] >> isClutter;
  tree["process"]["detection"]["enable"] >> isDetection;
  tree["process"]["tracker"]["enable"] >> isTracker;
  if (!isDetection)
  {
    isTracker = false;
  }

  // setup output data
  bool saveMap, saveDetection, saveTiming, saveDiagnostic, saveZeroDoppler;
  tree["save"]["map"] >> saveMap;
  tree["save"]["detection"] >> saveDetection;
  tree["save"]["timing"] >> saveTiming;
  saveDiagnostic = saveMap || saveDetection || saveTiming;
  auto saveDiagnosticNode = tree["save"].find_child(c4::to_csubstr("diagnostic"));
  if (saveDiagnosticNode.valid())
  {
    saveDiagnosticNode >> saveDiagnostic;
  }
  saveZeroDoppler = false;
  auto zeroDopplerNode = tree["save"].find_child(c4::to_csubstr("zeroDopplerRow"));
  if (zeroDopplerNode.valid())
  {
    zeroDopplerNode >> saveZeroDoppler;
  }
  std::string savePath, saveMapPath, saveDetectionPath, saveTimingPath, cfarDebugPath;
  if (saveIq || saveMap || saveDetection || saveTiming || saveDiagnostic || saveZeroDoppler || cfarDebug)
  {
    char startTimeStr[16];
    struct timeval currentTime = {0, 0};
    gettimeofday(&currentTime, NULL);
    strftime(startTimeStr, 16, "%Y%m%d-%H%M%S", localtime(&currentTime.tv_sec));
    savePath = path + startTimeStr;
  }
  if (saveMap)
  {
    saveMapPath = savePath + ".map";
  }
  if (saveDetection)
  {
    saveDetectionPath = savePath + ".detection";
  }
  if (saveTiming)
  {
    saveTimingPath = savePath + ".timing";
  }
  if (cfarDebug)
  {
    cfarDebugPath = savePath + ".cfar_debug.jsonl";
    cfarDetector1D->set_debug_logging(true, cfarDebugPath);
  }

  std::unique_ptr<Diagnostic> diagnostic;
  if (saveDiagnostic)
  {
    diagnostic = std::make_unique<Diagnostic>(savePath);
  }
  cfarDetector1D->set_row_metrics_enabled(saveDiagnostic);
  uint64_t prevOverflow0 = 0;
  uint64_t prevOverflow1 = 0;

  // setup output timing
  uint64_t tStart = current_time_ms();
  Timing *timing = new Timing(tStart);
  std::vector<std::string> timing_name;
  std::vector<double> timing_time;
  std::string jsonTiming;
  std::vector<uint64_t> time;

  // setup output json
  std::string mapJson, detectionJson, jsonTracker, jsonIqData;

  // run process
  std::thread t2([&]{
      while (true)
      {
        buffer1->lock();
        buffer2->lock();
        if ((buffer1->get_length() > nSamples) && (buffer2->get_length() > nSamples))
        {
          time.push_back(current_time_us());
          uint64_t overflowCount0 = buffer1->get_overflow_count();
          uint64_t overflowCount1 = buffer2->get_overflow_count();
          // extract data from buffer (bulk read for performance)
          buffer1->read_front(bulkX.data(), nSamples);
          buffer2->read_front(bulkY.data(), nSamples);
          buffer1->unlock();
          buffer2->unlock();
          // Load into processing IqData objects
          for (uint32_t i = 0; i < nSamples; i++)
          {
            x->push_back(bulkX[i]);
            y->push_back(bulkY[i]);
          }
          timing_helper(timing_name, timing_time, time, "extract_buffer");

          uint64_t timestampMs = time[0] / 1000;
          uint64_t droppedCh0 = overflowCount0 - prevOverflow0;
          uint64_t droppedCh1 = overflowCount1 - prevOverflow1;
          prevOverflow0 = overflowCount0;
          prevOverflow1 = overflowCount1;
          
          // spectrum
          spectrumAnalyser->process(x);
          timing_helper(timing_name, timing_time, time, "spectrum");

          double clutterPowerBefore = 0.0;
          std::deque<std::complex<double>> xPreClutter;
          std::deque<std::complex<double>> yPreClutter;
          if (diagnostic)
          {
            xPreClutter = x->get_data();
            yPreClutter = y->get_data();
            clutterPowerBefore = zero_lag_power(xPreClutter, yPreClutter);
          }
          std::vector<double> zeroDopplerBeforeDb;
          
          // clutter filter
          if (isClutter)
          {
            if (!filter->process(x, y))
            {
              // Drop this CPI cleanly so the next iteration starts fresh.
              x->clear();
              y->clear();
              time.clear();
              timing_name.clear();
              timing_time.clear();
              continue;
            }
            timing_helper(timing_name, timing_time, time, "clutter_filter");
          }

          if (saveZeroDoppler && diagnostic)
          {
            IqData xBeforeMap(nSamples);
            IqData yBeforeMap(nSamples);
            for (const auto &sample : xPreClutter)
            {
              xBeforeMap.push_back(sample);
            }
            for (const auto &sample : yPreClutter)
            {
              yBeforeMap.push_back(sample);
            }
            Map<std::complex<double>> *mapBefore = ambiguity->process(&xBeforeMap, &yBeforeMap);
            mapBefore->set_metrics();
            zeroDopplerBeforeDb = zero_doppler_row_db(mapBefore);
          }

          double clutterPowerAfter = 0.0;
          if (diagnostic)
          {
            std::deque<std::complex<double>> xPostClutter = x->get_data();
            std::deque<std::complex<double>> yPostClutter = y->get_data();
            clutterPowerAfter = zero_lag_power(xPostClutter, yPostClutter);
          }
          
          // ambiguity process
          map = ambiguity->process(x, y);
          map->set_metrics();
          timing_helper(timing_name, timing_time, time, "ambiguity_processing");
          
          // detection process
          uint32_t nDetectionsRaw = 0;
          uint32_t nDetectionsCentroid = 0;
          uint32_t nDetectionsFinal = 0;
          if (isDetection)
          {
            detection1 = cfarDetector1D->process(map, timestampMs);
            detection2 = centroid->process(detection1.get());
            detection = interpolate->process(detection2.get(), map);
            nDetectionsRaw = static_cast<uint32_t>(detection1->get_nDetections());
            nDetectionsCentroid = static_cast<uint32_t>(detection2->get_nDetections());
            nDetectionsFinal = static_cast<uint32_t>(detection->get_nDetections());
            timing_helper(timing_name, timing_time, time, "detector");
          }

          // tracker process
          if (isTracker)
          {
            track = tracker->process(detection.get(), timestampMs);
            timing_helper(timing_name, timing_time, time, "tracker");
          }

          if (diagnostic)
          {
            diagnostic->set_map_metrics(map->noisePower, map->maxPower);
            diagnostic->set_clutter_power(clutterPowerBefore, clutterPowerAfter);
            diagnostic->set_detection_counts(nDetectionsRaw, nDetectionsCentroid, nDetectionsFinal);
            diagnostic->set_sample_drops(droppedCh0, droppedCh1);
            diagnostic->set_cfar_rows(cfarDetector1D->get_row_doppler(),
              cfarDetector1D->get_row_noise_floor(),
              cfarDetector1D->get_row_threshold(),
              cfarDetector1D->get_row_detection_count());

            int64_t syncOffsetSamples = 0;
            double syncSnrDb = 0.0;
            bool hasSyncMetrics = false;
            uint64_t sampleDropsDeviceCh0 = 0;
            uint64_t sampleDropsDeviceCh1 = 0;
            if (capture->device)
            {
              hasSyncMetrics = capture->device->get_sync_metrics(syncOffsetSamples, syncSnrDb);
              if (capture->device->get_sample_drop_metrics(sampleDropsDeviceCh0, sampleDropsDeviceCh1))
              {
                diagnostic->set_sample_drops(sampleDropsDeviceCh0, sampleDropsDeviceCh1);
              }
            }
            diagnostic->set_sync_metrics(syncOffsetSamples, syncSnrDb, hasSyncMetrics);

            if (isDetection && detection)
            {
              diagnostic->update_persistence(detection->get_delay(), detection->get_doppler(),
                detection->get_snr(), timestampMs);
            }
            else
            {
              const std::vector<double> empty;
              diagnostic->update_persistence(empty, empty, empty, timestampMs);
            }

            diagnostic->log_cpi(timestampMs);

            if (saveZeroDoppler)
            {
              diagnostic->log_zero_doppler_row(timestampMs, zeroDopplerBeforeDb, zero_doppler_row_db(map));
            }
          }

          // output IqData meta data
          if (streamIqData && socket_iqdata)
          {
            jsonIqData = x->to_json(timestampMs);
            socket_iqdata->sendData(jsonIqData);
          }

          // output map data
          if (streamMap || saveMap)
          {
            mapJson = map->to_json(timestampMs);
            mapJson = map->delay_bin_to_km(mapJson, fs);
          }
          if (saveMap)
          {
            map->save(mapJson, saveMapPath);
          }
          if (streamMap && socket_map)
          {
            socket_map->sendData(mapJson);
          }

          // output detection data
          if (isDetection && (streamDetection || saveDetection))
          {
            detectionJson = detection->to_json(timestampMs);
            detectionJson = detection->delay_bin_to_km(detectionJson, fs);
          }
          if (isDetection && streamDetection && socket_detection)
          {
            socket_detection->sendData(detectionJson);
          }
          if (saveDetection && isDetection)
          {
            detection->save(detectionJson, saveDetectionPath);
          }

          // output tracker data
          if (isTracker && streamTrack && socket_track)
          {
            jsonTracker = track->to_json(timestampMs);
            socket_track->sendData(jsonTracker);
          }

          // output radar data timer
          timing_helper(timing_name, timing_time, time, "output_radar_data");

          // cpi timer
          time.push_back(current_time_us());
          double delta_ms = (double)(time.back()-time[0]) / 1000;
          timing_name.push_back("cpi");
          timing_time.push_back(delta_ms);
          std::cout << "CPI time (ms): " << delta_ms << std::endl;

          // output timing data
          timing->update(timestampMs, timing_time, timing_name);
          jsonTiming = timing->to_json();
          if (streamTiming && socket_timing)
          {
            socket_timing->sendData(jsonTiming);
          }
          if (saveTiming)
          {
            timing->save(jsonTiming, saveTimingPath);
          }
          timing_time.clear();
          timing_name.clear();

          // output CPI timestamp for updating data
          std::string t0_string = std::to_string(timestampMs);
          if (streamTimestamp && socket_timestamp)
          {
            socket_timestamp->sendData(t0_string);
          }
          time.clear();

        }
        else
        {
          buffer1->unlock();
          buffer2->unlock();
          // short delay to prevent tight looping
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
    });
  t2.join();
  t1.join();

  return 0;
}

void signal_callback_handler(int signum) {
  std::cout << "Caught signal " << signum << std::endl;
  if (CAPTURE_POINTER != nullptr)
  {
    CAPTURE_POINTER->device->kill();
  }
  else
  {
    exit(0);
  }
}

void getopt_print_help()
{
  std::cout << "--config <file.yml>: 	Set number of program\n"
               "--help:              	Show help\n";
  exit(1);
}

std::string getopt_process(int argc, char **argv)
{
  const char *const short_opts = "c:h";
  const option long_opts[] = {
      {"config", required_argument, nullptr, 'c'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, no_argument, nullptr, 0}};

  if (argc == 1)
  {
    std::cout << "Error: No arguments provided." << std::endl;
    exit(1);
  }

  std::string file;

  while (true)
  {
    const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

    // handle input "-", ":", etc
    if ((argc == 2) && (-1 == opt))
    {
      std::cout << "Error: No arguments provided." << std::endl;
      exit(1);
    }

    if (-1 == opt)
      break;

    switch (opt)
    {
    case 'c':
      file = std::string(optarg);
      break;

    case 'h':
      getopt_print_help();

    // unrecognised option
    case '?':
      exit(1);

    default:
      break;
    }
  }

  return file;
}

std::string ryml_get_file(const char *filename)
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (!in)
  {
    std::cerr << "could not open " << filename << std::endl;
    exit(1);
  }
  std::ostringstream contents;
  contents << in.rdbuf();
  return contents.str();
}

uint64_t current_time_ms()
{
  // current time in POSIX ms
  return std::chrono::duration_cast<std::chrono::milliseconds>
  (std::chrono::system_clock::now().time_since_epoch()).count();
}

uint64_t current_time_us()
{
  // current time in POSIX us
  return std::chrono::duration_cast<std::chrono::microseconds>
  (std::chrono::system_clock::now().time_since_epoch()).count();
}

void timing_helper(std::vector<std::string>& timing_name, 
  std::vector<double>& timing_time, std::vector<uint64_t>& time_us, 
  std::string name)
{
  time_us.push_back(current_time_us());
  double delta_ms = (double)(time_us.back()-time_us[time_us.size()-2]) / 1000;
  timing_name.push_back(name);
  timing_time.push_back(delta_ms);
}

double zero_lag_power(const std::deque<std::complex<double>> &x,
  const std::deque<std::complex<double>> &y)
{
  size_t n = std::min(x.size(), y.size());
  if (n == 0)
  {
    return 0.0;
  }

  std::complex<double> corr{0.0, 0.0};
  for (size_t i = 0; i < n; i++)
  {
    corr += y[i] * std::conj(x[i]);
  }
  corr /= static_cast<double>(n);
  return std::norm(corr);
}

std::vector<double> zero_doppler_row_db(Map<std::complex<double>> *map)
{
  std::vector<double> rowDb;
  if (!map || map->get_nRows() == 0)
  {
    return rowDb;
  }

  uint32_t zeroIdx = map->doppler_hz_to_bin(0.0);
  std::vector<std::complex<double>> row = map->get_row(zeroIdx);
  rowDb.reserve(row.size());
  for (const auto &value : row)
  {
    rowDb.push_back(10.0 * std::log10(std::abs(value) + 1e-30) - map->noisePower);
  }
  return rowDb;
}
