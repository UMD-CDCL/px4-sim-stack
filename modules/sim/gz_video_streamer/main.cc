// gz_video_streamer - Gazebo camera topics to H.264 video streams.
//
// This program subscribes to gz-transport image topics, encodes each one with
// GStreamer, and publishes it to the video router. It is a plain gz-transport
// client, not a gz-sim system plugin. That keeps the video path independent of
// the world file, of PX4, and of the vehicle model.
//
// PX4 ships a similar plugin, but that one binds to the first camera it finds.
// This program handles one stream for each camera.
//
// Usage:
//   gz_video_streamer --sink-base rtsp://video-router:8554 \
//       --stream name=gimbal,regex=.*/camera_link/sensor/camera/image$,bitrate=4000,fps=30
//
// Copyright (c) 2026. BSD 3-Clause, to match the PX4 plugin it takes its
// pipeline shape from.

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_run{true};

void OnSignal(int) { g_run = false; }

// Split "a=1,b=2" into pairs.
std::vector<std::pair<std::string, std::string>> ParseKeyValues(const std::string &s) {
  std::vector<std::pair<std::string, std::string>> out;
  std::string item;
  std::stringstream ss(s);
  while (std::getline(ss, item, ',')) {
    const auto eq = item.find('=');
    if (eq == std::string::npos) continue;
    out.emplace_back(item.substr(0, eq), item.substr(eq + 1));
  }
  return out;
}

bool StartsWith(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }

bool HaveFactory(const char *name) {
  GstElementFactory *f = gst_element_factory_find(name);
  if (f == nullptr) return false;
  gst_object_unref(f);
  return true;
}

// GStreamer caps string for a gz pixel format. An unsupported format returns
// an empty string, and the stream stays down with one log line.
const char *GstFormatFor(gz::msgs::PixelFormatType t) {
  switch (t) {
    case gz::msgs::PixelFormatType::RGB_INT8:  return "RGB";
    case gz::msgs::PixelFormatType::BGR_INT8:  return "BGR";
    case gz::msgs::PixelFormatType::RGBA_INT8: return "RGBA";
    case gz::msgs::PixelFormatType::BGRA_INT8: return "BGRA";
    case gz::msgs::PixelFormatType::L_INT8:    return "GRAY8";
    default: return nullptr;
  }
}

struct Spec {
  std::string name;
  std::string regex;
  std::string url;
  int bitrate_kbps = 4000;
  int fps = 30;
};

// One camera. It owns a gz subscription and a GStreamer pipeline.
class Stream {
 public:
  Stream(Spec spec, std::string encoder)
      : spec_(std::move(spec)), encoder_(std::move(encoder)), pattern_(spec_.regex) {}

  ~Stream() { Teardown(); }

  const Spec &spec() const { return spec_; }
  bool bound() const { return bound_; }

  // Look for a topic that matches, and subscribe to the first one.
  bool TryBind(const std::vector<std::string> &topics) {
    if (bound_) return true;
    for (const auto &t : topics) {
      if (!std::regex_search(t, pattern_)) continue;
      if (!node_.Subscribe(t, &Stream::OnImage, this)) {
        std::cerr << "[" << spec_.name << "] subscribe failed: " << t << std::endl;
        return false;
      }
      topic_ = t;
      bound_ = true;
      std::cout << "[" << spec_.name << "] bound to " << t << std::endl;
      std::cout << "[" << spec_.name << "] publishing to " << spec_.url << std::endl;
      return true;
    }
    return false;
  }

  // Restart the pipeline if GStreamer reported an error. The video router can
  // go down and come back, and the stream must recover on its own.
  void ServiceBus() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pipeline_ == nullptr) return;
    GstBus *bus = gst_element_get_bus(pipeline_);
    if (bus == nullptr) return;
    GstMessage *msg = gst_bus_timed_pop_filtered(
        bus, 0, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (msg == nullptr) return;

    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
      GError *err = nullptr;
      gchar *dbg = nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      std::cerr << "[" << spec_.name << "] pipeline error: "
                << (err != nullptr ? err->message : "unknown") << std::endl;
      if (err != nullptr) g_error_free(err);
      g_free(dbg);
    } else {
      std::cerr << "[" << spec_.name << "] pipeline reached end of stream" << std::endl;
    }
    gst_message_unref(msg);
    TeardownLocked();  // The next frame rebuilds it.
  }

  void Teardown() {
    std::lock_guard<std::mutex> lock(mutex_);
    TeardownLocked();
  }

 private:
  void TeardownLocked() {
    if (pipeline_ == nullptr) return;
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    appsrc_ = nullptr;
  }

  std::string SinkFragment() const {
    const std::string &u = spec_.url;
    if (StartsWith(u, "rtsp://")) {
      // MediaMTX accepts an RTSP ANNOUNCE. TCP avoids packet loss on the
      // docker bridge and costs almost nothing at these bitrates.
      return "rtspclientsink protocols=tcp latency=0 location=" + u;
    }
    if (StartsWith(u, "rtmp://")) {
      return "flvmux streamable=true ! rtmpsink location=" + u;
    }
    if (StartsWith(u, "udp://")) {
      const std::string hostport = u.substr(6);
      const auto colon = hostport.rfind(':');
      const std::string host = hostport.substr(0, colon);
      const std::string port = hostport.substr(colon + 1);
      return "rtph264pay config-interval=1 pt=96 ! udpsink sync=false host=" + host +
             " port=" + port;
    }
    std::cerr << "[" << spec_.name << "] unknown sink scheme in " << u
              << ", falling back to fakesink" << std::endl;
    return "fakesink";
  }

  bool BuildLocked(int width, int height, const char *format) {
    std::ostringstream p;
    p << "appsrc name=src is-live=true do-timestamp=true format=time block=false"
      << " ! queue leaky=downstream max-size-buffers=4"
      << " ! videoconvert"
      << " ! videorate"
      << " ! video/x-raw,format=I420,framerate=" << spec_.fps << "/1"
      << " ! " << encoder_
      << " ! h264parse config-interval=1"
      << " ! " << SinkFragment();

    const std::string desc = p.str();
    GError *err = nullptr;
    pipeline_ = gst_parse_launch(desc.c_str(), &err);
    if (pipeline_ == nullptr) {
      std::cerr << "[" << spec_.name << "] cannot build the pipeline: "
                << (err != nullptr ? err->message : "unknown") << std::endl;
      if (err != nullptr) g_error_free(err);
      return false;
    }
    if (err != nullptr) g_error_free(err);

    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw", "format", G_TYPE_STRING, format, "width", G_TYPE_INT, width, "height",
        G_TYPE_INT, height, "framerate", GST_TYPE_FRACTION, spec_.fps, 1, nullptr);
    g_object_set(G_OBJECT(appsrc_), "caps", caps, nullptr);
    gst_caps_unref(caps);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      std::cerr << "[" << spec_.name << "] cannot start the pipeline" << std::endl;
      TeardownLocked();
      return false;
    }
    std::cout << "[" << spec_.name << "] " << width << "x" << height << " " << format
              << " at " << spec_.fps << " fps -> " << spec_.url << std::endl;
    return true;
  }

  void OnImage(const gz::msgs::Image &msg) {
    const char *format = GstFormatFor(msg.pixel_format_type());
    if (format == nullptr) {
      if (!warned_format_) {
        std::cerr << "[" << spec_.name << "] unsupported pixel format "
                  << msg.pixel_format_type() << std::endl;
        warned_format_ = true;
      }
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (pipeline_ == nullptr) {
      const auto now = std::chrono::steady_clock::now();
      if (now < retry_after_) return;  // Do not rebuild on every frame.
      retry_after_ = now + std::chrono::seconds(2);
      if (!BuildLocked(msg.width(), msg.height(), format)) return;
    }

    const auto &data = msg.data();
    GstBuffer *buf = gst_buffer_new_allocate(nullptr, data.size(), nullptr);
    gst_buffer_fill(buf, 0, data.data(), data.size());
    if (gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf) != GST_FLOW_OK) {
      std::cerr << "[" << spec_.name << "] push failed, restarting the pipeline"
                << std::endl;
      TeardownLocked();
    }
  }

  Spec spec_;
  std::string encoder_;
  std::regex pattern_;
  gz::transport::Node node_;
  std::string topic_;
  bool bound_ = false;
  bool warned_format_ = false;

  std::mutex mutex_;
  GstElement *pipeline_ = nullptr;
  GstElement *appsrc_ = nullptr;
  std::chrono::steady_clock::time_point retry_after_{};
};

void Usage() {
  std::cout
      << "gz_video_streamer - Gazebo cameras to H.264 streams\n\n"
      << "  --sink-base URL   Base URL. A stream without an explicit url gets\n"
      << "                    <base>/<name>. Example: rtsp://video-router:8554\n"
      << "  --stream SPEC     Repeatable. Comma-separated key=value pairs:\n"
      << "                      name     stream name, also the URL path\n"
      << "                      regex    matches the gz image topic\n"
      << "                      url      overrides the sink URL\n"
      << "                      bitrate  kbit/s, default 4000\n"
      << "                      fps      default 30\n"
      << "  --encoder FRAG    GStreamer encoder fragment. Overrides the\n"
      << "                    automatic choice between nvh264enc and x264enc.\n"
      << "  --no-cuda         Force the software encoder.\n";
}

}  // namespace

int main(int argc, char **argv) {
  gst_init(&argc, &argv);
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  std::string sink_base = "rtsp://video-router:8554";
  std::string encoder_override;
  bool cuda = true;
  std::vector<Spec> specs;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help" || a == "-h") { Usage(); return 0; }
    if (a == "--no-cuda") { cuda = false; continue; }
    if (i + 1 >= argc) { std::cerr << "missing value for " << a << std::endl; return 2; }
    const std::string v = argv[++i];
    if (a == "--sink-base") { sink_base = v; continue; }
    if (a == "--encoder") { encoder_override = v; continue; }
    if (a == "--stream") {
      Spec s;
      for (const auto &kv : ParseKeyValues(v)) {
        if (kv.first == "name") s.name = kv.second;
        else if (kv.first == "regex") s.regex = kv.second;
        else if (kv.first == "url") s.url = kv.second;
        else if (kv.first == "bitrate") s.bitrate_kbps = std::atoi(kv.second.c_str());
        else if (kv.first == "fps") s.fps = std::atoi(kv.second.c_str());
      }
      if (s.name.empty() || s.regex.empty()) {
        std::cerr << "a --stream needs at least name= and regex=" << std::endl;
        return 2;
      }
      if (s.url.empty()) s.url = sink_base + "/" + s.name;
      specs.push_back(std::move(s));
      continue;
    }
    std::cerr << "unknown option " << a << std::endl;
    return 2;
  }

  if (specs.empty()) { Usage(); return 2; }

  // One encoder choice for every stream. NVENC keeps the CPU free for the
  // physics loop, which matters more than the small quality difference.
  const bool have_nvenc = cuda && HaveFactory("nvh264enc");
  std::cout << "encoder: " << (encoder_override.empty()
                                   ? (have_nvenc ? "nvh264enc (NVENC)" : "x264enc (software)")
                                   : encoder_override)
            << std::endl;

  std::vector<std::unique_ptr<Stream>> streams;
  for (auto &s : specs) {
    std::string enc = encoder_override;
    if (enc.empty()) {
      enc = have_nvenc
                ? "nvh264enc bitrate=" + std::to_string(s.bitrate_kbps) + " gop-size=30"
                : "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30 bitrate=" +
                      std::to_string(s.bitrate_kbps);
    }
    streams.push_back(std::make_unique<Stream>(s, enc));
  }

  gz::transport::Node discovery;
  int quiet_ticks = 0;
  while (g_run) {
    std::vector<std::string> topics;
    discovery.TopicList(topics);

    bool all_bound = true;
    for (auto &s : streams) {
      if (!s->bound()) {
        s->TryBind(topics);
        all_bound = all_bound && s->bound();
      }
      s->ServiceBus();
    }

    if (!all_bound && ++quiet_ticks % 15 == 0) {
      for (const auto &s : streams) {
        if (!s->bound()) {
          std::cout << "[" << s->spec().name << "] waiting for a topic that matches "
                    << s->spec().regex << std::endl;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "stopping" << std::endl;
  streams.clear();
  gst_deinit();
  return 0;
}
