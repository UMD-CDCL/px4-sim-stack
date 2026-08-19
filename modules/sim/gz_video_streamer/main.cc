// gz_video_streamer - Gazebo camera topics to H.265 video streams.
//
// This program subscribes to gz-transport image topics, encodes each one with
// GStreamer, and publishes it to the video router. It is a plain gz-transport
// client, not a gz-sim system plugin. That keeps the video path independent of
// the world file, of PX4, and of the vehicle model.
//
// PX4 ships a similar plugin, but that one binds to the first camera it finds.
// This program handles one stream for each camera.
//
// The fielded aircraft encodes H.265 CBR with nvv4l2h265enc, so this does the
// same wherever the machine can. A stream may also carry a width and a height,
// which rescales the encoder input: one camera render then feeds a full stream
// and the low-rate stream that crosses the radio link, at one encode each and
// no second render.
//
// Usage:
//   gz_video_streamer --sink-base rtsp://video-router:8554 \
//       --stream name=rgb1,regex=.*/camera_link/sensor/camera/image$,bitrate=8000,fps=15 \
//       --stream name=rgbl1,regex=.*/camera_link/sensor/camera/image$,bitrate=1000,fps=15,width=640,height=360
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

// Whether an encoder fragment can encode on this machine, which is a different
// question from whether it is installed. nvh264enc loads and reports "H.264
// encoding supported" on any NVIDIA GPU, then fails at set_format with
// "Selected preset not supported" on a driver that has dropped the preset
// GUIDs it asks for. The camera streams then stay down and the only clue is a
// GStreamer error with no encoder named in it.
//
// So run the candidate rather than trusting the registry: push a few frames
// through the real element and keep only what reaches end of stream. The shape
// below matches BuildLocked, because negotiation is part of what is being
// tested and a probe that converts differently answers a different question.
bool EncoderWorks(const std::string &fragment, const char *parser) {
  const std::string desc =
      "videotestsrc num-buffers=5 ! video/x-raw,format=RGB,width=640,height=480"
      " ! queue ! videoconvert n-threads=2 ! videorate drop-only=true ! videoscale"
      " ! video/x-raw,format={ NV12, I420 },framerate=15/1 ! " +
      fragment + " ! " + parser + " ! fakesink";

  GError *err = nullptr;
  GstElement *pipeline = gst_parse_launch(desc.c_str(), &err);
  if (err != nullptr) g_error_free(err);
  if (pipeline == nullptr) return false;

  bool ok = false;
  if (gst_element_set_state(pipeline, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE) {
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(
        bus, 3 * GST_SECOND,
        static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    ok = msg != nullptr && GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS;
    if (msg != nullptr) gst_message_unref(msg);
    gst_object_unref(bus);
  }
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  return ok;
}

// The encoders to try, best first. Each one takes bitrate in kbit/s, which is
// added when the fragment is built, and names the parser its output needs.
//
// H.265 first, because that is what the aircraft sends. The H.264 entries are
// the floor: an RTSP reader picks its depayloader from the caps it discovers,
// so a machine with no H.265 encoder still serves every consumer.
struct EncoderChoice {
  const char *element;
  const char *options;
  const char *parser;
  bool needs_cuda;
  const char *label;
};

const EncoderChoice kEncoders[] = {
    // The current NVENC elements. They take the p1-p7 presets with a separate
    // tune, which is the interface a recent driver still accepts.
    {"nvcudah265enc",
     "gop-size=30 rate-control=cbr tune=low-latency zero-reorder-delay=true b-frames=0",
     "h265parse", true, "NVENC H.265"},
    // The older NVENC elements, for a GStreamer that predates the ones above.
    // Their presets are the deprecated GUIDs, so newer hardware rejects them
    // and the probe moves on.
    {"nvh265enc", "gop-size=30", "h265parse", true, "NVENC H.265, legacy element"},
    {"x265enc", "tune=zerolatency speed-preset=ultrafast key-int-max=30",
     "h265parse", false, "software H.265"},
    {"nvcudah264enc",
     "gop-size=30 rate-control=cbr tune=low-latency zero-reorder-delay=true b-frames=0",
     "h264parse", true, "NVENC H.264"},
    {"nvh264enc", "gop-size=30", "h264parse", true, "NVENC H.264, legacy element"},
    {"x264enc", "tune=zerolatency speed-preset=ultrafast key-int-max=30",
     "h264parse", false, "software H.264"},
};

std::string EncoderFragment(const EncoderChoice &c, int bitrate_kbps) {
  return std::string(c.element) + " bitrate=" + std::to_string(bitrate_kbps) + " " + c.options;
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
  // 0 keeps the size the camera renders.
  int width = 0;
  int height = 0;
};

// One camera. It owns a gz subscription and a GStreamer pipeline.
class Stream {
 public:
  Stream(Spec spec, std::string encoder, std::string parser)
      : spec_(std::move(spec)), encoder_(std::move(encoder)), parser_(std::move(parser)),
        pattern_(spec_.regex) {}

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
      const std::string payloader = parser_ == "h265parse" ? "rtph265pay" : "rtph264pay";
      return payloader + " config-interval=1 pt=96 ! udpsink sync=false host=" + host +
             " port=" + port;
    }
    std::cerr << "[" << spec_.name << "] unknown sink scheme in " << u
              << ", falling back to fakesink" << std::endl;
    return "fakesink";
  }

  bool BuildLocked(int width, int height, const char *format) {
    std::ostringstream p;
    // Two formats rather than one. Pinning I420 suits the software encoders
    // and rules out the NVENC ones, which take NV12 in system memory only;
    // leaving the format free lets negotiation settle on Y444, and the stream
    // then carries High 4:4:4 Predictive, which no NVDEC decodes. DeepStream
    // reports that as "Feature not supported on this GPU" against the decoder,
    // several containers away from the encoder that chose it.
    //
    // The pair below is what every consumer can read: 8 bit, 4:2:0.
    p << "appsrc name=src is-live=true do-timestamp=true format=time block=false"
      << " ! queue leaky=downstream max-size-buffers=4"
      << " ! videoconvert n-threads=2"
      << " ! videorate drop-only=true"
      << " ! videoscale"
      << " ! video/x-raw,format={ NV12, I420 },framerate=" << spec_.fps << "/1";
    if (spec_.width > 0 && spec_.height > 0) {
      p << ",width=" << spec_.width << ",height=" << spec_.height;
    }
    p << " ! " << encoder_
      << " ! " << parser_ << " config-interval=1"
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
    std::cout << "[" << spec_.name << "] " << width << "x" << height << " " << format;
    if (spec_.width > 0 && spec_.height > 0) {
      std::cout << " scaled to " << spec_.width << "x" << spec_.height;
    }
    std::cout << " at " << spec_.fps << " fps -> " << spec_.url << std::endl;
    return true;
  }

  void OnImage(const gz::msgs::Image &msg) {
    ++frames_;

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
    GstBuffer *buf = gst_buffer_new_memdup(data.data(), data.size());
    if (gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf) != GST_FLOW_OK) {
      std::cerr << "[" << spec_.name << "] push failed, restarting the pipeline"
                << std::endl;
      TeardownLocked();
    }
  }

  // Gazebo puts the camera's own frame counter in the header key/value pairs.
  // Prefer it over our own count: it survives a reconnect and it matches what
  // a Gazebo log shows. Fall back to counting if the field is absent.
  Spec spec_;
  std::string encoder_;
  std::string parser_;
  std::regex pattern_;
  uint64_t frames_ = 0;
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
      << "gz_video_streamer - Gazebo cameras to H.265 streams\n\n"
      << "  --sink-base URL   Base URL. A stream without an explicit url gets\n"
      << "                    <base>/<name>. Example: rtsp://video-router:8554\n"
      << "  --stream SPEC     Repeatable. Comma-separated key=value pairs:\n"
      << "                      name     stream name, also the URL path\n"
      << "                      regex    matches the gz image topic\n"
      << "                      url      overrides the sink URL\n"
      << "                      bitrate  kbit/s, default 4000\n"
      << "                      fps      default 30\n"
      << "                      width    rescale before encoding, with height\n"
      << "                      height   rescale before encoding, with width\n"
      << "  --encoder FRAG    GStreamer encoder fragment. Skips the probe that\n"
      << "                    picks between the H.265 and H.264 encoders.\n"
      << "  --parser NAME     Parser for --encoder output. Default h265parse.\n"
      << "  --no-cuda         Force the software encoder.\n";
}

}  // namespace

int main(int argc, char **argv) {
  gst_init(&argc, &argv);
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  std::string sink_base = "rtsp://video-router:8554";
  std::string encoder_override;
  std::string parser_override = "h265parse";
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
    if (a == "--parser") { parser_override = v; continue; }
    if (a == "--stream") {
      Spec s;
      for (const auto &kv : ParseKeyValues(v)) {
        if (kv.first == "name") s.name = kv.second;
        else if (kv.first == "regex") s.regex = kv.second;
        else if (kv.first == "url") s.url = kv.second;
        else if (kv.first == "bitrate") s.bitrate_kbps = std::atoi(kv.second.c_str());
        else if (kv.first == "fps") s.fps = std::atoi(kv.second.c_str());
        else if (kv.first == "width") s.width = std::atoi(kv.second.c_str());
        else if (kv.first == "height") s.height = std::atoi(kv.second.c_str());
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
  //
  // Each candidate is run before it is chosen, because a GPU encoder can be
  // installed and still refuse the job. The probe costs a second or two once,
  // against camera streams that otherwise fail with no encoder named.
  const EncoderChoice *chosen = nullptr;
  if (encoder_override.empty()) {
    const int probe_bitrate = specs.front().bitrate_kbps;
    for (const auto &c : kEncoders) {
      if (c.needs_cuda && !cuda) continue;
      if (!HaveFactory(c.element)) continue;
      if (!EncoderWorks(EncoderFragment(c, probe_bitrate), c.parser)) {
        std::cerr << "encoder: " << c.element
                  << " is installed but cannot encode here, trying the next one" << std::endl;
        continue;
      }
      chosen = &c;
      break;
    }
    if (chosen == nullptr) {
      // Nothing passed, software included. Take the software encoder anyway:
      // its own error is more use than a stream that never starts.
      chosen = &kEncoders[sizeof(kEncoders) / sizeof(kEncoders[0]) - 1];
      std::cerr << "encoder: no candidate passed its probe, falling back to " << chosen->element
                << std::endl;
    }
    std::cout << "encoder: " << chosen->element << " (" << chosen->label << ")" << std::endl;
  } else {
    std::cout << "encoder: " << encoder_override << " (from --encoder)" << std::endl;
  }

  std::vector<std::unique_ptr<Stream>> streams;
  for (auto &s : specs) {
    std::string enc = encoder_override;
    std::string parser = parser_override;
    if (enc.empty()) {
      enc = EncoderFragment(*chosen, s.bitrate_kbps);
      parser = chosen->parser;
    }
    streams.push_back(std::make_unique<Stream>(s, enc, parser));
  }

  gz::transport::Node discovery;
  int quiet_ticks = 0;
  while (g_run) {
    bool all_bound = true;
    for (const auto &s : streams) all_bound = all_bound && s->bound();

    // The topic list only serves binding, and streams never unbind. Once
    // every stream is bound, stop asking discovery for it.
    if (!all_bound) {
      std::vector<std::string> topics;
      discovery.TopicList(topics);
      all_bound = true;
      for (auto &s : streams) {
        if (!s->bound()) s->TryBind(topics);
        all_bound = all_bound && s->bound();
      }
    }

    for (auto &s : streams) {
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
