//ip_cam_superpoint.cpp

#include <iostream>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <filesystem>
#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>
#include "SuperPointFast.h"
#include "utils/ThreadSafeQueue.h"
#include "utils/FeatureExtractorTypes.h"
#include "PipelinedFE/PipelinedProcessFactory.h"
#include "SuperPointFast.h"

using namespace std;
using namespace cv;

// Atomic variables for FPS calculation
std::atomic<float> current_fps(0.0f);
std::atomic<int> frame_count(0);
std::atomic<int> keypoints_count(0);

// Extern declaration for the global atomic variables
extern std::atomic<float> g_conf_thresh;
extern std::atomic<int> g_dist_thresh;
// Global atomic variable for target FPS
std::atomic<int> g_target_fps(10);

// Trackbar variables (scaled for better usability)
int conf_thresh_trackbar = 5; // Range 0-100, maps to 0.0-0.03
int dist_thresh_trackbar = 2;  // Range 0-10
int target_fps_trackbar = 10;  // Range 1-60

// Callback functions for trackbars
void on_conf_thresh_change(int pos, void *)
{
  float new_val = pos * 0.0003f; // Scale from 0-100 to 0.0-0.03
  g_conf_thresh.store(new_val);
  std::cout << "Confidence threshold set to: " << new_val << std::endl;
}

void on_dist_thresh_change(int pos, void *)
{
  g_dist_thresh.store(pos);
  std::cout << "NMS distance threshold set to: " << pos << std::endl;
}

void on_target_fps_change(int pos, void *)
{
  g_target_fps.store(pos);
  std::cout << "Target FPS set to: " << pos << std::endl;
}

void print_usage(char *prog_name)
{
  std::cout << "Usage: " << prog_name << " [options] vocabulary_path settings_path model_path ip_address port target_fps protocol [camera_fps]" << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  -t <num>      Number of preprocessing/postprocessing threads to use (default: 2)" << std::endl;
  std::cout << "                Note: DPU runners fixed at 2" << std::endl;
  std::cout << "  -o <dir>      Output directory for trajectory files (optional)" << std::endl;
  std::cout << "Example: " << prog_name << " -t 4 /path/to/vocabulary.fbow /path/to/settings.yaml /path/to/superpoint.xmodel 192.168.1.100 8080 10 rtsp" << std::endl;
  std::cout << std::endl;
  std::cout << "Environment variables:" << std::endl;
  std::cout << "  DEBUG_FEAT                Enable detailed feature debug output" << std::endl;
  std::cout << "  DEBUG_KeypointVisualization   Enable keypoint visualization" << std::endl;
  std::cout << "  SKIP_SLAM                 Skip SLAM initialization and processing" << std::endl;
}

int main(int argc, char *argv[])
{
  // Default parameters
  int num_threads = 2;
  std::string output_dir = "./";
  std::string vocabulary_path;
  std::string settings_path;
  std::string model_path;
  std::string ip_address;
  int port = 0;
  int target_fps = 10;
  std::string protocol = "rtsp"; // Default protocol
  int camera_fps = 20;           // Default camera FPS

  // Parse command line arguments
  int arg_index = 1;
  while (arg_index < argc)
  {
    std::string arg = argv[arg_index];
    if (arg == "-t")
    {
      if (arg_index + 1 < argc)
      {
        num_threads = std::stoi(argv[arg_index + 1]);
        arg_index += 2;
      }
      else
      {
        std::cerr << "Error: Missing value for -t option" << std::endl;
        print_usage(argv[0]);
        return 1;
      }
    }
    else if (arg == "-o")
    {
      if (arg_index + 1 < argc)
      {
        output_dir = std::string(argv[arg_index + 1]);
        // Ensure output directory has trailing slash
        if (output_dir.back() != '/')
        {
          output_dir += '/';
        }
        arg_index += 2;
      }
      else
      {
        std::cerr << "Error: Missing value for -o option" << std::endl;
        print_usage(argv[0]);
        return 1;
      }
    }
    else if (arg == "-h" || arg == "--help")
    {
      print_usage(argv[0]);
      return 0;
    }
    else
    {
      break;
    }
  }

  // Get positional arguments
  if (arg_index + 6 < argc)
  {
    vocabulary_path = argv[arg_index];
    settings_path = argv[arg_index + 1];
    model_path = argv[arg_index + 2];
    ip_address = argv[arg_index + 3];
    port = std::stoi(argv[arg_index + 4]);
    target_fps = std::stoi(argv[arg_index + 5]);
    g_target_fps.store(target_fps); // Initialize the atomic variable
    protocol = argv[arg_index + 6];

    // Check if camera_fps is provided as an argument
    if (arg_index + 7 < argc)
    {
      camera_fps = std::stoi(argv[arg_index + 7]);
    }
  }
  else
  {
    std::cerr << "Error: Missing required positional arguments" << std::endl;
    print_usage(argv[0]);
    return 1;
  }

  // Print configuration
  std::cout << "Configuration:" << std::endl;
  std::cout << "- Vocabulary path: " << vocabulary_path << std::endl;
  std::cout << "- Settings path: " << settings_path << std::endl;
  std::cout << "- Number of pre/post-processing threads: " << num_threads << std::endl;
  std::cout << "- Number of DPU runners: 2 (fixed)" << std::endl;
  std::cout << "- Model: " << model_path << std::endl;
  std::cout << "- IP Address: " << ip_address << std::endl;
  std::cout << "- Port: " << port << std::endl;
  std::cout << "- Target FPS: " << target_fps << std::endl;
  std::cout << "- Protocol: " << protocol << std::endl;
  std::cout << "- Camera FPS: " << camera_fps << std::endl;
  std::cout << "- Output directory: " << output_dir << std::endl;

  // Check for SKIP_SLAM environment variable
  bool skip_slam = (std::getenv("SKIP_SLAM") != nullptr);
  if (skip_slam)
  {
    std::cout << "SKIP_SLAM environment variable detected. SLAM system will not be initialized." << std::endl;
  }

  try
  {
    // SLAM initialization time
    float imageScale = 1.0f;
    ORB_SLAM3::System *SLAM = nullptr;

    if (!skip_slam)
    {
      std::chrono::steady_clock::time_point SLAM_start = std::chrono::steady_clock::now();

      // Create SLAM system - initializes all system threads
      SLAM = new ORB_SLAM3::System(vocabulary_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);

      std::chrono::steady_clock::time_point SLAM_end = std::chrono::steady_clock::now();
      float SLAM_load_time = std::chrono::duration_cast<std::chrono::duration<double>>(SLAM_end - SLAM_start).count() * 1000.0;
      cout << "Loaded SLAM in " << SLAM_load_time / 1000.0 << " s" << endl;

      imageScale = SLAM->GetImageScale();
    }

    // Queue sizes - slightly larger than the demo to accommodate SLAM processing
    const size_t INPUT_QUEUE_SIZE = 10;
    const size_t OUTPUT_QUEUE_SIZE = 10;

    // Create thread-safe queues for our pipeline
    ThreadSafeQueue<ORB_SLAM3::InputQueueItem> inputQueue(INPUT_QUEUE_SIZE);
    ThreadSafeQueue<ORB_SLAM3::ResultQueueItem> outputQueue(OUTPUT_QUEUE_SIZE);

    // Create the real-time SuperPoint feature extractor
    auto featureProcessor = ORB_SLAM3::PipelinedProcessFactory::CreatePipelinedProcess(
        ORB_SLAM3::FeatureExtractorType::SUPERPOINT,
        inputQueue,
        outputQueue,
        "", // Empty feature directory to use real-time extraction
        model_path,
        num_threads);

    // Start the processor thread
    featureProcessor->StartProcessing();

    // Create a window for displaying results and controls
    cv::namedWindow("ORB-SLAM3 with SuperPoint", cv::WINDOW_NORMAL);

    // Initialize trackbar values based on current atomic values
    conf_thresh_trackbar = g_conf_thresh.load() / 0.0003f; // Scale for UI
    dist_thresh_trackbar = g_dist_thresh.load();
    target_fps_trackbar = g_target_fps.load();

    // Create trackbars for parameter adjustments
    cv::createTrackbar("Conf Thresh", "ORB-SLAM3 with SuperPoint",
                       &conf_thresh_trackbar, 100, on_conf_thresh_change);
    cv::createTrackbar("NMS Dist Thresh", "ORB-SLAM3 with SuperPoint",
                       &dist_thresh_trackbar, 10, on_dist_thresh_change);
    cv::createTrackbar("Target FPS", "ORB-SLAM3 with SuperPoint",
                       &target_fps_trackbar, 60, on_target_fps_change);

    // Initialize trackbars with current values to trigger callbacks
    on_conf_thresh_change(conf_thresh_trackbar, nullptr);
    on_dist_thresh_change(dist_thresh_trackbar, nullptr);
    on_target_fps_change(target_fps_trackbar, nullptr);

    // URL for video stream based on protocol
    std::string stream_url = protocol + "://" + ip_address + ":" + std::to_string(port);

    if (protocol == "rtsp")
    {
      stream_url += "/h264.sdp";
    }
    else if (protocol == "http")
    {
      stream_url += "/video";
    }
    std::cout << "Connecting to stream: " << stream_url << std::endl;

    // Start producer thread to capture frames from IP camera
    std::thread producer_thread([stream_url, &inputQueue, camera_fps, imageScale, SLAM]()
                                {
      // Open video stream
      cv::VideoCapture cap(stream_url);
      if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open video stream at " << stream_url << std::endl;
        inputQueue.shutdown();
        return;
      }

      // Clear the buffer and get the most recent frame
      // First, set buffer size to 1 to minimize buffering
      cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

      size_t frame_idx = 0;
      size_t frames_dropped = 0;
      size_t total_frames_received = 0; // Count of all frames from camera, including flushed and dropped
      auto last_frame_time = std::chrono::high_resolution_clock::now();
      int target_fps, flush_fps;
      float flush_ratio;
      float flush_counter = 0.0;
      
      while (true) {
        // Get current target_fps from atomic variable
        target_fps = g_target_fps.load();
        flush_fps = camera_fps - target_fps;
        
        if (flush_fps > 0) {
          flush_ratio = (float)flush_fps / (float)camera_fps;
        } else {
          flush_ratio = 0.0;
        }
        
        // Calculate time since last frame
        auto now = std::chrono::high_resolution_clock::now();
        auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time);
        
        // Enforce target FPS by waiting if needed
        int frame_time_ms = 1000 / target_fps;
        if (frame_duration.count() < frame_time_ms) {
          std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - frame_duration.count()));
          now = std::chrono::high_resolution_clock::now();
        }
        
        last_frame_time = now;
        
        // Capture frame
        cv::Mat frame;
        
        // Flush any existing frames in the buffer to get the latest frame
        flush_counter += flush_ratio;
        if(flush_counter >= 1.0) {
          cap.grab();
          total_frames_received++; // Count flushed frame
          flush_counter -= 1.0;
          flush_counter += flush_ratio;
        }
        
        // Now retrieve the latest frame
        if (!cap.read(frame)) {
            if (frame.empty()) {
                // Try once more
                std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms / 4));
                cap >> frame;
                if (frame.empty()) {
                  std::cerr << "End of stream or error in video capture" << std::endl;
                  break;
                }
            }
        }
        
        // Count every successfully received frame
        total_frames_received++;

        if(SLAM != nullptr) {
        // Resize image if needed for SLAM
          if(imageScale != 1.f) {
              int width = frame.cols * imageScale;
                int height = frame.rows * imageScale;
                cv::resize(frame, frame, cv::Size(width, height));
          }else if(SLAM->settings_ && SLAM->settings_->needToResize()) {
              cv::resize(frame, frame, SLAM->settings_->newImSize());
          }
        }
        
        // Create input queue item
        ORB_SLAM3::InputQueueItem item;
        item.index = frame_idx++;
        
        // Calculate timestamp based on total frames from camera and camera FPS for consistent timing
        // This accounts for all frames (including flushed and dropped ones) for accurate timing
        double timestamp = static_cast<double>(total_frames_received) / camera_fps;
        item.timestamp = timestamp;
        item.image = frame;
        item.filename = "frame_" + std::to_string(item.index);
        
        // Try to add to queue, but don't block if queue is full (drop frames instead)
        if (!inputQueue.try_enqueue_for(item, std::chrono::milliseconds(1))) {
          // Frame dropped - queue is full
          frames_dropped++;
          frame_idx--;
        }
      }
      
      // Signal that no more frames will be added
      inputQueue.shutdown();
      std::cout << "Producer thread finished, received " << total_frames_received << " frames, processed " << frame_idx 
                << " frames, dropped " << frames_dropped << " frames" << std::endl; });

    // Consumer thread - process results from the feature extraction and run SLAM

    int processedFrames = 0;
    ORB_SLAM3::ResultQueueItem result;

    // For FPS and latency calculation
    std::deque<std::chrono::steady_clock::time_point> frameTimes;
    const int fpsWindowSize = 30; // Calculate FPS over last 30 frames
    double dequeueLatency = 0;
    double trackLatency = 0;
    double currentFps = 0;

    // For enforcing target FPS in the consumer thread
    auto last_process_time = std::chrono::high_resolution_clock::now();
    int frame_time_ms = 1000 / g_target_fps.load();
    int wait_time = 0;

    std::chrono::steady_clock::time_point dequeueStart = std::chrono::steady_clock::now();

    while (outputQueue.dequeue(result))
    {
      std::chrono::steady_clock::time_point dequeueEnd = std::chrono::steady_clock::now();
      dequeueLatency = std::chrono::duration_cast<std::chrono::duration<double>>(dequeueEnd - dequeueStart).count() * 1000.0;
      
      // Get latest target FPS
      int current_target_fps = g_target_fps.load();
      frame_time_ms = 1000 / current_target_fps;
      
      // Calculate time since last frame processing
      auto now = std::chrono::high_resolution_clock::now();
      auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_process_time);

      // Enforce target FPS by waiting if needed
      if (frame_duration.count() < frame_time_ms)
      {
        wait_time = frame_time_ms - frame_duration.count();
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
        now = std::chrono::high_resolution_clock::now();
      }else{
        wait_time = 0;
      }

      last_process_time = now;

      // Update frame times for FPS calculation
      frameTimes.push_back(dequeueEnd);
      if (frameTimes.size() > fpsWindowSize)
      {
        frameTimes.pop_front();
      }

      // Calculate current FPS if we have enough frames
      if (frameTimes.size() >= 2)
      {
        double duration = std::chrono::duration_cast<std::chrono::duration<double>>(
                              frameTimes.back() - frameTimes.front())
                              .count();
        currentFps = (frameTimes.size() - 1) / duration;
      }

      // Count keypoints
      keypoints_count.store(result.keypoints.size());

      // Track using SLAM if not skipped
      if (!skip_slam && SLAM != nullptr)
      {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        SLAM->TrackMonocular(result);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        trackLatency = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000.0;
      }
      else
      {
        trackLatency = 0.0;
      }

      // Only show keypoints if DEBUG_KeypointVisualization is set
      if (std::getenv("DEBUG_KeypointVisualization"))
      {
        // Use the original image from the result for display
        cv::Mat display_img = result.image.clone();

        // Draw keypoints on the image
        for (const auto &kp : result.keypoints)
        {
          cv::circle(display_img,
                     cv::Point(kp.pt.x, kp.pt.y),
                     3, cv::Scalar(0, 255, 0), -1); // green dots for keypoints
        }

        // Add performance metrics to the image
        std::string fpsText = "FPS: " + std::to_string(static_cast<int>(currentFps + 0.5));
        std::string dequeueText = "Dequeue: " + std::to_string(static_cast<int>(dequeueLatency + 0.5)) + " ms";
        std::string trackText = "Track: " + std::to_string(static_cast<int>(trackLatency + 0.5)) + " ms";
        std::string featuresText = "#Feat: " + std::to_string(result.keypoints.size());
        std::string imageIdText = "ImID: " + std::to_string(result.index);

        // Add threshold information
        std::string confText = "Conf: " + std::to_string(g_conf_thresh.load()).substr(0, 6);
        std::string nmsText = "NMS: " + std::to_string(g_dist_thresh.load());

        cv::putText(display_img, fpsText, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 0, 0), 2);
        cv::putText(display_img, dequeueText, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 0, 0), 2);
        cv::putText(display_img, trackText, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 0, 0), 2);
        cv::putText(display_img, confText, cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 0, 0), 2);
        cv::putText(display_img, nmsText, cv::Point(20, 150), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 0, 0), 2);

        // Position text in bottom right corner
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.75;
        int thickness = 2;
        cv::Size featTextSize = cv::getTextSize(featuresText, fontFace, fontScale, thickness, nullptr);
        cv::Size idTextSize = cv::getTextSize(imageIdText, fontFace, fontScale, thickness, nullptr);

        // Calculate positions for upper right corner
        cv::Point featTextPos(display_img.cols - featTextSize.width - 20,
                              30); // 30 pixels from top
        cv::Point idTextPos(display_img.cols - idTextSize.width - 20,
                            30 + featTextSize.height + 10); // 10 pixels spacing between texts

        // Draw text in upper right
        cv::putText(display_img, featuresText, featTextPos, fontFace, fontScale, cv::Scalar(255, 0, 0), thickness);
        cv::putText(display_img, imageIdText, idTextPos, fontFace, fontScale, cv::Scalar(255, 0, 0), thickness);

        // Display the image with keypoints
        cv::imshow("ORB-SLAM3 with SuperPoint", display_img);

        // Process key events - allow for parameter adjustments with keyboard
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27)
        { // 'q' or ESC key
          std::cout << "User requested exit" << std::endl;
          outputQueue.shutdown();
          break;
        }
      }

      // Debug print for features
#ifdef DEBUG_PRINT
      if (std::getenv("DEBUG_FEAT") != nullptr)
      {
        // Print the FPS, dequeue latency, and track latency
        std::cout << "[DEBUG] frame=" << result.index
                  << " | kps=" << result.keypoints.size()
                  << " | FPS=" << std::fixed << std::setprecision(1) << currentFps
                  << " | Dequeue=" << std::fixed << std::setprecision(1) << dequeueLatency << "ms"
                  << " | Track=" << std::fixed << std::setprecision(1) << trackLatency << "ms"
                  << " | Wait=" << std::fixed << std::setprecision(1) << wait_time << "ms"
                  << std::endl;
      }
#endif

      processedFrames++;

      // Print progress every 50 frames
      if (processedFrames % 50 == 0)
      {
        cout << "Processed " << processedFrames << " frames" << endl;
      }

      dequeueStart = std::chrono::steady_clock::now();
    }

    std::cout << "Consumer thread finished" << std::endl;
    cv::destroyAllWindows();

    // Wait for threads to complete
    producer_thread.join();

    // Stop the processor
    featureProcessor->Stop();

    // Save camera trajectory and shutdown SLAM if not skipped
    if (!skip_slam && SLAM != nullptr)
    {
      const string kf_file = output_dir + "KeyFrameTrajectory.txt";
      const string f_file = output_dir + "CameraTrajectory.txt";
      SLAM->SaveTrajectoryEuRoC(f_file);
      SLAM->SaveKeyFrameTrajectoryEuRoC(kf_file);

      // Stop all threads
      SLAM->Shutdown();

      // Clean up SLAM object
      delete SLAM;
    }

    std::cout << "SLAM processing completed successfully!" << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}