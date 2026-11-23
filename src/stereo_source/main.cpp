#include <gst/allocators/gstfdmemory.h>
#include <gst/allocators/gstshmallocator.h>
#include <gst/base/gstaggregator.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>

#include <dvrk_data/config.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <dvrk_data/stereo_common.hpp>

namespace {

struct CommandLineOptions {
  std::string config_file;
  bool stereo_output = false;
};

void print_usage(const char *executable) {
  std::cerr << "Usage: " << executable << " -c <config.json> [--stereo-output]" << std::endl;
}

bool parse_arguments(int argc, char *argv[], CommandLineOptions &options) {
  bool seen_config = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ros-args") {
      break;
    }

    if (arg == "--stereo-output" && !options.stereo_output) {
      options.stereo_output = true;
      continue;
    }

    if (arg == "-c" && i + 1 < argc) {
      if (seen_config) {
        std::cerr << "Error: multiple -c arguments are not supported."
                  << std::endl;
        return false;
      }
      options.config_file = argv[++i];
      seen_config = true;
      continue;
    }

    std::cerr << "Error: unknown argument '" << arg << "'." << std::endl;
    return false;
  }

  if (!seen_config) {
    std::cerr << "Error: exactly one config file is required." << std::endl;
    return false;
  }

  return true;
}

GstPadProbeReturn source_timestamp_probe_cb(GstPad *pad,
                                            GstPadProbeInfo *info,
                                            gpointer user_data) {
  (void)pad;
  if (!(info->type & GST_PAD_PROBE_TYPE_BUFFER)) {
    return GST_PAD_PROBE_OK;
  }

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
  if (!gst_buffer_is_writable(buf)) {
    buf = gst_buffer_make_writable(buf);
    GST_PAD_PROBE_INFO_DATA(info) = buf;
  }

  DcFrameTimestamps timestamps = dc_buffer_get_frame_timestamps(buf);
  const gint64 now = dc_clock_realtime_ns();
  const int side = GPOINTER_TO_INT(user_data);
  if (side == 0) {
    timestamps.left_source_ts = now;
  } else {
    timestamps.right_source_ts = now;
  }
  dc_buffer_set_frame_timestamps(buf, timestamps);
  return GST_PAD_PROBE_OK;
}

void add_timestamp_probe(GstElement *pipeline, const std::string &element_name,
                         int side) {
  GstElement *element =
      gst_bin_get_by_name(GST_BIN(pipeline), element_name.c_str());
  if (element == nullptr) {
    return;
  }

  GstPad *pad = gst_element_get_static_pad(element, "src");
  if (pad != nullptr) {
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                      source_timestamp_probe_cb, GINT_TO_POINTER(side),
                      nullptr);
    gst_object_unref(pad);
  }
  gst_object_unref(element);
}

struct StereoMix {
  GstElement *element = nullptr;
  GstPad *left = nullptr;
  GstPad *right = nullptr;
  std::mutex caps_mutex;
  int width[2]{};
  int height[2]{};
  DcFrameTimestamps selected;
  bool output_caps_ready = false;
};

GstPadProbeReturn stereo_mix_caps(GstPad *pad, GstPadProbeInfo *info, gpointer data) {
  auto &mix = *static_cast<StereoMix *>(data);
  GstEvent *event = GST_PAD_PROBE_INFO_EVENT(info);
  const int eye = pad == mix.left ? 0 : 1;
  if (GST_EVENT_TYPE(event) == GST_EVENT_STREAM_START) {
    std::lock_guard<std::mutex> lock(mix.caps_mutex);
    mix.width[eye] = mix.height[eye] = 0;
  }
  if (GST_EVENT_TYPE(event) != GST_EVENT_CAPS) return GST_PAD_PROBE_OK;

  GstCaps *caps = nullptr;
  gst_event_parse_caps(event, &caps);
  GstVideoInfo video;
  if (!gst_video_info_from_caps(&video, caps)) return GST_PAD_PROBE_OK;

  std::lock_guard<std::mutex> lock(mix.caps_mutex);
  const int other = 1 - eye;
  if (video.width < 2 || video.height < 2 || video.width % 2 || video.height % 2 ||
      video.width > std::numeric_limits<int>::max() / 2 ||
      (mix.width[other] && (video.width != mix.width[other] || video.height != mix.height[other]))) {
    GST_ELEMENT_ERROR(mix.element, CORE, NEGOTIATION,
                      ("Side-by-side stereo needs equal, even per-eye dimensions"),
                      ("Incoming size %dx%d", video.width, video.height));
    return GST_PAD_PROBE_DROP;
  }
  mix.width[eye] = video.width;
  mix.height[eye] = video.height;
  g_object_set(mix.right, "xpos", video.width, nullptr);
  return GST_PAD_PROBE_OK;
}

void stereo_samples_selected(GstAggregator *aggregator, GstSegment *, guint64,
                             guint64, guint64, GstStructure *, gpointer data) {
  auto &mix = *static_cast<StereoMix *>(data);
  auto timestamps = [&](GstPad *pad) {
    GstSample *sample = gst_aggregator_peek_next_sample(aggregator, GST_AGGREGATOR_PAD(pad));
    if (!sample) return DcFrameTimestamps{};
    const auto result = dc_buffer_get_frame_timestamps(gst_sample_get_buffer(sample));
    gst_sample_unref(sample);
    return result;
  };
  mix.selected.left_source_ts = timestamps(mix.left).left_source_ts;
  mix.selected.right_source_ts = timestamps(mix.right).right_source_ts;
}

GstPadProbeReturn stereo_mix_output(GstPad *, GstPadProbeInfo *info, gpointer data) {
  auto &mix = *static_cast<StereoMix *>(data);
  if (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM) {
    GstEvent *event = GST_PAD_PROBE_INFO_EVENT(info);
    if (GST_EVENT_TYPE(event) == GST_EVENT_STREAM_START) mix.output_caps_ready = false;
    if (GST_EVENT_TYPE(event) != GST_EVENT_CAPS) return GST_PAD_PROBE_OK;

    // With auto sizing, compositor may offer placeholder caps before either
    // input is negotiated. Publish only the actual side-by-side dimensions.
    GstCaps *caps = nullptr;
    gst_event_parse_caps(event, &caps);
    GstVideoInfo video;
    std::lock_guard<std::mutex> lock(mix.caps_mutex);
    const int eye = mix.width[0] ? 0 : 1;
    mix.output_caps_ready = mix.width[eye] && gst_video_info_from_caps(&video, caps) &&
        video.width == 2 * mix.width[eye] && video.height == mix.height[eye];
    return mix.output_caps_ready ? GST_PAD_PROBE_OK : GST_PAD_PROBE_DROP;
  }
  if (!mix.output_caps_ready) return GST_PAD_PROBE_DROP;
  auto *buffer = gst_buffer_make_writable(GST_PAD_PROBE_INFO_BUFFER(info));
  GST_PAD_PROBE_INFO_DATA(info) = buffer;
  dc_buffer_set_frame_timestamps(buffer, mix.selected);
  return GST_PAD_PROBE_OK;
}

void attach_stereo_mix(GstElement *pipeline, StereoMix &mix) {
  mix.element = gst_bin_get_by_name(GST_BIN(pipeline), "source_mix");
  mix.left = gst_element_get_static_pad(mix.element, "sink_0");
  mix.right = gst_element_get_static_pad(mix.element, "sink_1");
  for (GstPad *pad : {mix.left, mix.right}) {
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM, stereo_mix_caps, &mix, nullptr);
  }
  g_signal_connect(mix.element, "samples-selected", G_CALLBACK(stereo_samples_selected), &mix);
  GstPad *src = gst_element_get_static_pad(mix.element, "src");
  gst_pad_add_probe(src, static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER |
                    GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM), stereo_mix_output, &mix, nullptr);
  gst_object_unref(src);
}

GstPadProbeReturn ensure_fd_memory(GstPad *pad, GstPadProbeInfo *info, gpointer data) {
  auto *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
  const guint count = gst_buffer_n_memory(buffer);
  guint index = 0;
  while (index < count && gst_is_fd_memory(gst_buffer_peek_memory(buffer, index))) ++index;
  if (index == count) return GST_PAD_PROBE_OK;

  // An already-I420 eye can pass through videoconvert without an allocation.
  // A tee cannot propagate unixfdsink's allocator back to the shared source.
  const gsize size = gst_buffer_get_size(buffer);
  auto *memory = gst_allocator_alloc(GST_ALLOCATOR(data), size, nullptr);
  GstMapInfo mapped;
  if (memory && gst_memory_map(memory, &mapped, GST_MAP_WRITE)) {
    const auto copied = gst_buffer_extract(buffer, 0, mapped.data, size);
    gst_memory_unmap(memory, &mapped);
    if (copied == size) {
      auto *shared = gst_buffer_new();
      gst_buffer_append_memory(shared, memory);
      gst_buffer_copy_into(shared, buffer, GST_BUFFER_COPY_METADATA, 0, -1);
      gst_buffer_unref(buffer);
      GST_PAD_PROBE_INFO_DATA(info) = shared;
      return GST_PAD_PROBE_OK;
    }
  }
  if (memory) gst_memory_unref(memory);
  auto *sink = gst_pad_get_parent_element(pad);
  GST_ELEMENT_ERROR(sink, RESOURCE, WRITE, ("Cannot share stereo eye frame via Unix FD"), (nullptr));
  gst_object_unref(sink);
  return GST_PAD_PROBE_DROP;
}

void attach_eye_socket_allocator(GstElement *pipeline, const char *name) {
  auto *sink = gst_bin_get_by_name(GST_BIN(pipeline), name);
  if (!sink) return;
  auto *pad = gst_element_get_static_pad(sink, "sink");
  gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, ensure_fd_memory,
                    gst_shm_allocator_get(), gst_object_unref);
  gst_object_unref(pad);
  gst_object_unref(sink);
}

std::string build_branch(const dvrk_data::SourceConfig &eye, const std::string &name,
                         const std::string &mix_input, const std::string &dimensions) {
  const std::string queue =
      " ! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream";
  std::string branch = eye.gst_input + " ! queue name=__" + name + "_src_q__"
      " max-size-buffers=3 max-size-time=0 max-size-bytes=0 leaky=downstream";

  if (eye.gst_output.empty() && mix_input.empty()) return branch + " ! fakesink sync=false";
  const std::string convert = " ! videoconvert ! video/x-raw,format=I420" + dimensions;
  if (mix_input.empty()) {
    return branch + convert + queue + " ! " + dvrk_gst::build_sink(eye.gst_output);
  }
  if (eye.gst_output.empty()) return branch + convert + " ! " + mix_input;

  // The socket branch negotiates its FD-backed allocator independently.
  const std::string tee = "source_" + name;
  return branch + " ! tee name=" + tee + " " +
         tee + "." + queue + convert + " ! " + dvrk_gst::build_sink(eye.gst_output) +
         " name=" + name + "_eye_socket " +
         tee + "." + queue + convert + " ! " + mix_input;
}

std::string build_pipeline_string(const dvrk_data::AppConfig &cfg) {
  const bool combine = !cfg.stereo.gst_output.empty();
  const std::string dimensions = combine && cfg.original_width > 0 && cfg.original_height > 0
      ? ",width=" + std::to_string(cfg.original_width) + ",height=" + std::to_string(cfg.original_height)
      : "";
  std::string pipeline = build_branch(cfg.left, "left", combine ? "source_mix.sink_0" : "", dimensions) +
      " " + build_branch(cfg.right, "right", combine ? "source_mix.sink_1" : "", dimensions);
  if (combine) {
    pipeline += " compositor name=source_mix background=1 emit-signals=true"
                " ! video/x-raw,format=I420"
                " ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream ! " +
                dvrk_gst::build_sink(cfg.stereo.gst_output);
  }
  return pipeline;
}

}  // namespace

int main(int argc, char *argv[]) {
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  dc_frame_timestamps_meta_register();

  CommandLineOptions options;
  if (!parse_arguments(argc, argv, options)) {
    print_usage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("stereo_source");
  const std::string &path = options.config_file;
  if (!std::filesystem::exists(path)) {
    RCLCPP_ERROR(node->get_logger(), "Config file does not exist: %s",
                 path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  Json::Value root;
  if (!dvrk_data::Config::load_from_file(path, root)) {
    rclcpp::shutdown();
    return 1;
  }

  if (!dvrk_data::Config::check_type(root, "dvrk_data:stereo_source@1.0.0", path)) {
    rclcpp::shutdown();
    return 1;
  }

  dvrk_data::AppConfig cfg;
  try {
    cfg = dvrk_data::Config::parse_app_config(root);
    const auto &camera = static_cast<const Json::Value &>(root)["camera"];
    if (camera.isObject() && camera.isMember("rectification")) {
      throw std::runtime_error("Use a separate stereo_rectify.json for camera rectification");
    }
    if (root.isMember("gst_output") &&
        (!root["gst_output"].isString() || dvrk_gst::resolve(cfg.stereo.gst_output).empty())) {
      throw std::runtime_error("Root gst_output must name a @dvrk:role:name stereo socket");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  if (cfg.left.gst_input.empty() || cfg.right.gst_input.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Config '%s' must define camera.left.gst_input and "
                 "camera.right.gst_input",
                 cfg.name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (options.stereo_output && !cfg.stereo.gst_output_specified) {
    cfg.stereo.gst_output = dvrk_gst::make(dvrk_gst::ROLE_STEREO_SOURCE, "stereo");
  }
  // Preserve the per-eye defaults for existing alignment configurations.
  if (cfg.stereo.gst_output.empty()) {
    if (!cfg.left.gst_output_specified)
      cfg.left.gst_output = dvrk_gst::make(dvrk_gst::ROLE_STEREO_SOURCE, "left");
    if (!cfg.right.gst_output_specified)
      cfg.right.gst_output = dvrk_gst::make(dvrk_gst::ROLE_STEREO_SOURCE, "right");
  }
  cfg.left.gst_input = dvrk_gst::build_input(
      cfg.left.gst_input, dvrk_gst::ROLE_STEREO_SOURCE,
      cfg.original_width, cfg.original_height);
  cfg.right.gst_input = dvrk_gst::build_input(
      cfg.right.gst_input, dvrk_gst::ROLE_STEREO_SOURCE,
      cfg.original_width, cfg.original_height);

  for (const auto &output : {cfg.left.gst_output, cfg.right.gst_output}) {
    if (output.empty()) {
      continue;
    }
    if (dvrk_gst::resolve(output).empty()) {
      RCLCPP_ERROR(node->get_logger(), "Invalid gst_output socket reference: %s",
                   output.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(node->get_logger(), "left gst output: %s", cfg.left.gst_output.c_str());
  RCLCPP_INFO(node->get_logger(), "right gst output: %s", cfg.right.gst_output.c_str());
  if (!cfg.stereo.gst_output.empty()) {
    RCLCPP_INFO(node->get_logger(), "combined gst output: %s", cfg.stereo.gst_output.c_str());
  }

  dc_stereo::warn_if_interlaced_stream(cfg.left.gst_input, node->get_logger(),
                                       "camera.left");
  dc_stereo::warn_if_interlaced_stream(cfg.right.gst_input, node->get_logger(),
                                       "camera.right");

  const std::string pipeline_string = build_pipeline_string(cfg);
  if (!dc_stereo::validate_pipeline(pipeline_string, node->get_logger(),
                                    "stereo_source")) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "GStreamer pipeline string:\n%s",
              pipeline_string.c_str());

  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(pipeline_string.c_str(), &error);
  if (error != nullptr || pipeline == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create pipeline: %s",
                 error && error->message ? error->message : "unknown");
    if (error != nullptr) {
      g_error_free(error);
    }
    if (pipeline != nullptr) {
      gst_object_unref(pipeline);
    }
    rclcpp::shutdown();
    return 1;
  }

  StereoMix mix;
  if (!cfg.stereo.gst_output.empty()) {
    attach_stereo_mix(pipeline, mix);
    gst_shm_allocator_init_once();
    attach_eye_socket_allocator(pipeline, "left_eye_socket");
    attach_eye_socket_allocator(pipeline, "right_eye_socket");
  }
  add_timestamp_probe(pipeline, "__left_src_q__", 0);
  add_timestamp_probe(pipeline, "__right_src_q__", 1);

  const int status = dc_stereo::run_pipeline(
      pipeline, node, "stereo_source",
      "Stereo source background pipeline started");
  gst_object_unref(pipeline);
  if (mix.element) {
    gst_object_unref(mix.left);
    gst_object_unref(mix.right);
    gst_object_unref(mix.element);
  }
  rclcpp::shutdown();
  return status;
}
