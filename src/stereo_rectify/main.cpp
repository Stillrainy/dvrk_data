#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <iostream>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>

#include <dvrk_data/config.hpp>
#include <dvrk_data/cpu_timestamp_meta.hpp>
#include <dvrk_data/dvrk_gst_socket.hpp>
#include <dvrk_data/stereo_common.hpp>

#include "calibration.hpp"

namespace {

struct Options {
  std::string path;
  std::string input;
};

void usage(const char *program) {
  std::cout << "Usage: " << program << " -c <stereo_rectify.json> [-i <side-by-side input>]\n";
}

Options parse_options(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ros-args") break;
    if ((arg == "-c" || arg == "--config") && i + 1 < argc && options.path.empty()) {
      options.path = argv[++i];
    } else if ((arg == "-i" || arg == "--input") && i + 1 < argc && options.input.empty()) {
      options.input = argv[++i];
    } else {
      throw std::runtime_error("Unexpected argument: " + arg);
    }
  }
  if (options.path.empty()) throw std::runtime_error("Missing -c <stereo_rectify.json>");
  return options;
}

std::string read_endpoint(const Json::Value &root, const char *key, const std::string &fallback,
                          bool allow_pipeline) {
  const auto &field = root[key];
  if (!root.isMember(key)) return fallback;
  if (!field.isString() || (allow_pipeline && field.asString().empty())) {
    throw std::runtime_error(std::string("Invalid ") + key);
  }
  const auto value = field.asString();
  const std::regex socket("^@dvrk:[A-Za-z0-9_.-]+:[A-Za-z0-9_.-]+$");
  if (value.empty() || (allow_pipeline && value.front() != '@') ||
      std::regex_match(value, socket)) return value;
  throw std::runtime_error(std::string(key) + " must use a canonical @dvrk:role:name socket");
}

struct RectifyConfig {
  dvrk_data::StereoRectification calibration;
  std::string input;
  std::string output;
};

dvrk_data::StereoRectification read_calibration(const Json::Value &root,
                                                const std::filesystem::path &directory) {
  const bool has_file = root.isMember("calibration_file");
  const bool has_pair = root.isMember("camera_info");
  if (has_file == has_pair) {
    throw std::runtime_error("Provide either calibration_file or a camera_info pair");
  }
  auto resolve_path = [&](const Json::Value &field, const char *name) {
    if (!field.isString() || field.asString().empty()) {
      throw std::runtime_error(std::string(name) + " must name a calibration file");
    }
    // Open the configured path on every start so it can follow an updated symlink.
    return directory / field.asString();
  };
  if (has_file) {
    return dvrk_data::load_stereo_rectification(resolve_path(root["calibration_file"], "calibration_file"));
  }
  const auto &pair = root["camera_info"];
  if (!pair.isObject()) throw std::runtime_error("camera_info must specify left and right YAML files");
  return dvrk_data::load_stereo_camera_info(resolve_path(pair["left"], "camera_info.left"),
                                            resolve_path(pair["right"], "camera_info.right"));
}

RectifyConfig load_config(const Options &options) {
  Json::Value root;
  if (!dvrk_data::Config::load_from_file(options.path, root) || !root.isObject() ||
      !dvrk_data::Config::check_type(root, "dvrk_data:stereo_rectify@1.0.0", options.path)) {
    throw std::runtime_error("Cannot load stereo_rectify configuration");
  }
  if (root.get("backend", "gpu") != Json::Value("gpu")) {
    throw std::runtime_error("Set backend to 'gpu' for OpenGL rectification");
  }
  auto calibration = read_calibration(root, std::filesystem::path(options.path).parent_path());
  const Json::Value &camera = static_cast<const Json::Value &>(root)["camera"];
  if (root.isMember("preserve_size") ||
      (!camera.isNull() && (!camera.isObject() || camera.isMember("left") ||
        camera.isMember("right") || camera.isMember("crop") || camera.isMember("alignment")))) {
    throw std::runtime_error("stereo_rectify takes one side-by-side input and uses the saved calibration geometry");
  }
  const auto &size = camera["size"];
  if (!size.isNull() && size != Json::Value("auto") &&
      (!size.isObject() || !size["width"].isInt() || !size["height"].isInt() ||
       size["width"].asInt() != calibration.width || size["height"].asInt() != calibration.height)) {
    throw std::runtime_error("camera.size must be 'auto' or match the stereo calibration");
  }
  if (!options.input.empty()) root["gst_input"] = options.input;
  return {
      std::move(calibration),
      read_endpoint(root, "gst_input", dvrk_gst::make(dvrk_gst::ROLE_STEREO_SOURCE, "stereo"), true),
      read_endpoint(root, "gst_output", dvrk_gst::make(dvrk_gst::ROLE_STEREO_RECTIFY, "stereo"), false)};
}

std::string build_pipeline(const RectifyConfig &config) {
  const std::string size = ",width=" + std::to_string(config.calibration.width * 2) +
                           ",height=" + std::to_string(config.calibration.height);
  const std::string queue =
      " ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream";
  // Convert to NV12 on the GPU, then let videoconvert write I420 into the
  // FD-backed buffers requested by unixfdsink.
  return dvrk_gst::build_input(config.input, dvrk_gst::ROLE_STEREO_RECTIFY) + queue +
         " ! glupload ! video/x-raw(memory:GLMemory),interlace-mode=progressive" + size +
         " ! glcolorconvert ! glshader name=rectify"
         " ! glcolorconvert ! video/x-raw(memory:GLMemory),format=NV12"
         " ! gldownload ! video/x-raw,format=NV12"
         " ! videoconvert name=rectify_output ! video/x-raw,format=I420" + size + queue + " ! " +
         (config.output.empty() ? "fakesink sync=false" : dvrk_gst::build_sink(config.output));
}

GstPadProbeReturn mark_output(GstPad *, GstPadProbeInfo *info, gpointer) {
  auto *buffer = gst_buffer_make_writable(GST_PAD_PROBE_INFO_BUFFER(info));
  GST_PAD_PROBE_INFO_DATA(info) = buffer;
  auto timestamps = dc_buffer_get_frame_timestamps(buffer);
  timestamps.stereo_output_ts = dc_clock_realtime_ns();
  dc_buffer_set_frame_timestamps(buffer, timestamps);
  return GST_PAD_PROBE_OK;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
    usage(argv[0]);
    return 0;
  }
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  dc_frame_timestamps_meta_register();

  const auto node = std::make_shared<rclcpp::Node>("stereo_rectify");
  int status = 1;
  try {
    const auto config = load_config(parse_options(argc, argv));
    if (config.input == config.output) {
      throw std::runtime_error("Rectification input and output sockets must differ");
    }
    const auto description = build_pipeline(config);
    GError *error = nullptr;
    std::unique_ptr<GstElement, decltype(&gst_object_unref)> pipeline(
        gst_parse_launch(description.c_str(), &error), gst_object_unref);
    if (error || !pipeline) {
      const std::string message = error ? error->message : "Cannot build rectification pipeline";
      g_clear_error(&error);
      throw std::runtime_error(message);
    }
    GstElement *shader = gst_bin_get_by_name(GST_BIN(pipeline.get()), "rectify");
    g_object_set(shader, "fragment", config.calibration.shader.c_str(), nullptr);
    gst_object_unref(shader);

    GstElement *output = gst_bin_get_by_name(GST_BIN(pipeline.get()), "rectify_output");
    GstPad *pad = gst_element_get_static_pad(output, "src");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, mark_output, nullptr, nullptr);
    gst_object_unref(pad);
    gst_object_unref(output);

    RCLCPP_INFO(node->get_logger(), "OpenGL stereo rectification at %dx%d per eye",
                config.calibration.width, config.calibration.height);
    RCLCPP_INFO(node->get_logger(), "GStreamer pipeline string:\n%s", description.c_str());
    status = dc_stereo::run_pipeline(pipeline.get(), node, "stereo_rectify",
                                    "Stereo rectification pipeline started");
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }
  rclcpp::shutdown();
  return status;
}
