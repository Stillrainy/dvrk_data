#ifndef DVRK_DATA_STEREO_RECTIFY_CALIBRATION_HPP
#define DVRK_DATA_STEREO_RECTIFY_CALIBRATION_HPP

#include <filesystem>
#include <string>

namespace dvrk_data {

struct StereoRectification {
  int width;
  int height;
  std::string shader;
};

StereoRectification load_stereo_rectification(const std::filesystem::path &path);
StereoRectification load_stereo_camera_info(const std::filesystem::path &left_path,
                                            const std::filesystem::path &right_path);

}  // namespace dvrk_data

#endif
