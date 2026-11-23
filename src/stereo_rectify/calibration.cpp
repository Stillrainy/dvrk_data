#include "calibration.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace dvrk_data {
namespace {

struct Eye {
  std::vector<double> camera;
  std::vector<double> distortion;
  std::vector<double> rotation;
  std::vector<double> projection;
};

YAML::Node read_calibration(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Cannot open stereo calibration");
  }
  std::string first_line;
  std::getline(input, first_line);
  // FileStorage uses a nonstandard YAML directive. The remaining document
  // uses ordinary YAML nodes, including the tagged rows/cols/data matrices.
  if (first_line != "%YAML:1.0" && first_line != "%YAML:1.0\r") {
    input.seekg(0);
  }
  return YAML::Load(input);
}

std::vector<double> read_matrix(const YAML::Node &root, const std::string &key,
                                int rows, int cols) {
  const auto matrix = root[key];
  const int actual_rows = matrix["rows"].as<int>();
  const int actual_cols = matrix["cols"].as<int>();
  const auto data = matrix["data"];
  const bool vector_shape = actual_rows == 1 || actual_cols == 1;
  if (actual_rows <= 0 || actual_cols <= 0 ||
      (rows ? actual_rows != rows || actual_cols != cols : !vector_shape) ||
      !data.IsSequence() ||
      data.size() != static_cast<size_t>(actual_rows) * actual_cols) {
    throw std::runtime_error("Invalid matrix shape for " + key);
  }
  auto values = data.as<std::vector<double>>();
  for (const double value : values) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
      throw std::runtime_error("Invalid numeric value in " + key);
    }
  }
  return values;
}

void validate_eye(const Eye &result, const std::string &model, const std::string &label) {
  const auto count = result.distortion.size();
  if (!((model == "plumb_bob" && count == 5) ||
        (model == "rational_polynomial" && (count == 8 || count == 14)))) {
    throw std::runtime_error("Unsupported distortion model or coefficient count for " + label);
  }
  const auto &k = result.camera;
  const auto &p = result.projection;
  if (!(k[0] > 0.0 && k[4] > 0.0 && p[0] > 0.0 && p[5] > 0.0) ||
      k[1] != 0.0 || k[3] != 0.0 || k[6] != 0.0 || k[7] != 0.0 || k[8] != 1.0 ||
      p[1] != 0.0 || p[4] != 0.0 || p[8] != 0.0 || p[9] != 0.0 ||
      p[10] != 1.0 || p[11] != 0.0) {
    throw std::runtime_error("Invalid camera or projection matrix for " + label);
  }

  const auto &r = result.rotation;
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = i; j < 3; ++j) {
      const double dot = r[3*i] * r[3*j] + r[3*i+1] * r[3*j+1] + r[3*i+2] * r[3*j+2];
      if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-4) {
        throw std::runtime_error("Invalid rectification rotation for " + label);
      }
    }
  }
}

Eye read_eye(const YAML::Node &root, const std::string &eye, const std::string &model) {
  Eye result{read_matrix(root, "K" + eye, 3, 3), read_matrix(root, "D" + eye, 0, 0),
             read_matrix(root, "R" + eye, 3, 3), read_matrix(root, "P" + eye, 3, 4)};
  validate_eye(result, model, "eye " + eye);
  return result;
}

void validate_dimensions(int width, int height) {
  if (width < 2 || height < 2 || width % 2 || height % 2 ||
      width > std::numeric_limits<int>::max() / 2) {
    throw std::runtime_error("Calibration must have positive even per-eye dimensions");
  }
}

struct CameraInfo {
  int width;
  int height;
  Eye eye;
};

CameraInfo read_camera_info(const std::filesystem::path &path) {
  try {
    const auto info = read_calibration(path);
    CameraInfo result{info["image_width"].as<int>(), info["image_height"].as<int>(),
                      {read_matrix(info, "camera_matrix", 3, 3),
                       read_matrix(info, "distortion_coefficients", 0, 0),
                       read_matrix(info, "rectification_matrix", 3, 3),
                       read_matrix(info, "projection_matrix", 3, 4)}};
    validate_dimensions(result.width, result.height);
    validate_eye(result.eye, info["distortion_model"].as<std::string>(), "CameraInfo");
    return result;
  } catch (const std::exception &error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

void write_eye_shader(std::ostringstream &glsl, const Eye &eye, const char *suffix) {
  const auto &k = eye.camera;
  const auto &p = eye.projection;
  const auto &r = eye.rotation;
  std::array<double, 14> d{};
  std::copy(eye.distortion.begin(), eye.distortion.end(), d.begin());
  // Write the rows of R into GLSL columns to produce the inverse rotation Rᵀ.
  // P's last column encodes the baseline and is used for triangulation.
  glsl << "vec2 map_" << suffix << "(vec2 pixel) {\n"
       << "  const mat3 inverse_rotation = mat3("
       << r[0] << "," << r[1] << "," << r[2] << ","
       << r[3] << "," << r[4] << "," << r[5] << ","
       << r[6] << "," << r[7] << "," << r[8] << ");\n"
       << "  const vec2 dst_center = vec2(" << p[2] << "," << p[6] << ");\n"
       << "  const vec2 dst_focal = vec2(" << p[0] << "," << p[5] << ");\n"
       << "  const vec2 src_center = vec2(" << k[2] << "," << k[5] << ");\n"
       << "  const vec2 src_focal = vec2(" << k[0] << "," << k[4] << ");\n"
       << "  const vec3 radial = vec3(" << d[0] << "," << d[1] << "," << d[4] << ");\n"
       << "  const vec3 denominator = vec3(" << d[5] << "," << d[6] << "," << d[7] << ");\n"
       << "  const vec2 tangential = vec2(" << d[2] << "," << d[3] << ");\n"
       << "  const vec2 prism_r2 = vec2(" << d[8] << "," << d[10] << ");\n"
       << "  const vec2 prism_r4 = vec2(" << d[9] << "," << d[11] << ");\n";
  glsl << R"glsl(
  vec3 ray = inverse_rotation * vec3((pixel - dst_center) / dst_focal, 1.0);
  if (ray.z <= 0.0) return vec2(-2.0);
  vec2 xy = ray.xy / ray.z;
  float r2 = dot(xy, xy);
  float ratio = (1.0 + r2 * (radial.x + r2 * (radial.y + r2 * radial.z))) /
      (1.0 + r2 * (denominator.x + r2 * (denominator.y + r2 * denominator.z)));
  vec2 distorted = xy * ratio + 2.0 * xy.x * xy.y * tangential +
      (r2 + 2.0 * xy * xy) * tangential.yx + r2 * (prism_r2 + r2 * prism_r4);
)glsl";
  if (d[12] != 0.0 || d[13] != 0.0) {
    const double cx = std::cos(d[12]), sx = std::sin(d[12]);
    const double cy = std::cos(d[13]), sy = std::sin(d[13]);
    // Project the distorted ray onto the tilted sensor, keeping the origin fixed.
    glsl << "  const mat3 tilt_rotation = mat3("
         << cy << ",0.0," << sy << "," << sy*sx << "," << cx << "," << -cy*sx
         << "," << -sy*cx << "," << sx << "," << cy*cx << ");\n"
         << "  vec3 tilted = tilt_rotation * vec3(distorted, 1.0);\n"
         << "  distorted = (" << cy*cx << " * tilted.xy - vec2("
         << -sy*cx << "," << sx << ") * tilted.z) / tilted.z;\n";
  }
  glsl << "  return src_focal * distorted + src_center;\n}\n";
}

std::string make_shader(const Eye &left, const Eye &right) {
  std::ostringstream glsl;
  glsl.imbue(std::locale::classic());
  glsl << std::scientific << std::setprecision(std::numeric_limits<float>::max_digits10);
  glsl << R"glsl(#ifdef GL_ES
precision highp float;
#endif
varying vec2 v_texcoord;
uniform sampler2D tex;
uniform float width;
uniform float height;
)glsl";
  write_eye_shader(glsl, left, "left");
  write_eye_shader(glsl, right, "right");
  glsl << R"glsl(
void main() {
  const vec4 black = vec4(0.0, 0.0, 0.0, 1.0);
  vec2 full_size = vec2(width, height);
  vec2 eye_size = vec2(width * 0.5, height);
  vec2 pixel = v_texcoord * full_size - vec2(0.5);
  bool right_eye = pixel.x >= eye_size.x - 0.5;
  vec2 offset = vec2(right_eye ? eye_size.x : 0.0, 0.0);
  vec2 local = pixel - offset;
  vec2 source = right_eye ? map_right(local) : map_left(local);
  if (!all(greaterThan(source, vec2(-1.0))) || !all(lessThan(source, eye_size))) {
    gl_FragColor = black;
    return;
  }
  // Clamp to this eye's texel centers and blend with black at the image edge.
  // The texture filter must never interpolate across the left/right seam.
  vec2 extent = eye_size - vec2(1.0);
  vec2 edge_weight = clamp(min(source + vec2(1.0), eye_size - source), 0.0, 1.0);
  vec2 uv = (clamp(source, vec2(0.0), extent) + offset + vec2(0.5)) / full_size;
  gl_FragColor = mix(black, texture2D(tex, uv), edge_weight.x * edge_weight.y);
}
)glsl";
  return glsl.str();
}

}  // namespace

StereoRectification load_stereo_rectification(const std::filesystem::path &path) {
  try {
    const auto root = read_calibration(path);
    if (root["type"].as<std::string>() != "dvrk_calibration:stereo@1.0.0" ||
        root["calibration_date"].as<std::string>().empty()) {
      throw std::runtime_error("Expected a dated dvrk_calibration:stereo@1.0.0 export");
    }
    const int width = root["image_width"].as<int>();
    const int height = root["image_height"].as<int>();
    validate_dimensions(width, height);
    read_matrix(root, "R", 3, 3);
    const auto baseline = read_matrix(root, "T", 0, 0);
    if (baseline.size() != 3 || std::hypot(baseline[0], baseline[1], baseline[2]) < 1e-9) {
      throw std::runtime_error("Calibration must have a nonzero three-component stereo baseline");
    }
    const auto model = root["distortion_model"].as<std::string>();
    return {width, height, make_shader(read_eye(root, "1", model), read_eye(root, "2", model))};
  } catch (const std::exception &error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

StereoRectification load_stereo_camera_info(const std::filesystem::path &left_path,
                                            const std::filesystem::path &right_path) {
  const auto left = read_camera_info(left_path);
  const auto right = read_camera_info(right_path);
  if (left.width != right.width || left.height != right.height) {
    throw std::runtime_error("Left and right CameraInfo image sizes must match");
  }
  const auto &p1 = left.eye.projection;
  const auto &p2 = right.eye.projection;
  if (std::hypot(p1[3] / p1[0] - p2[3] / p2[0],
                 p1[7] / p1[5] - p2[7] / p2[5]) < 1e-9) {
    throw std::runtime_error("CameraInfo projection matrices must encode a nonzero stereo baseline");
  }
  return {left.width, left.height, make_shader(left.eye, right.eye)};
}

}  // namespace dvrk_data
