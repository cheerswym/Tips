#include "onboard/vis/common/colormap.h"

#include <utility>

#include "absl/synchronization/mutex.h"
#include "glog/logging.h"
#include "onboard/utils/file_util.h"
#include "onboard/vis/common/proto/colormap.pb.h"

namespace qcraft {
namespace vis {

namespace {

class ColormapData {
 public:
  static const std::vector<Color> &GetData(const std::string &scheme) {
    absl::MutexLock l(&colormaps_mutex_);
    if (colormaps_.empty()) {
      static constexpr char kPredefinedColormapDataFile[] =
          "onboard/vis/common/data/predefined_colormaps.pb.txt";
      ColormapsProto proto;
      file_util::FileToProto(kPredefinedColormapDataFile, &proto);
      for (const auto &colormap_proto : proto.colormaps()) {
        std::vector<Color> &colormap = colormaps_[colormap_proto.scheme()];
        for (const auto &color_proto : colormap_proto.data()) {
          colormap.push_back(Color(color_proto));
        }
      }
    }
    const auto it = colormaps_.find(scheme);
    if (it == colormaps_.end()) {
      LOG(ERROR) << "Colormap scheme " << scheme
                 << " is not found. Returning default scheme.";
      return colormaps_[Colormap::kDefaultScheme];
    }
    return it->second;
  }

 private:
  static std::map<std::string, std::vector<Color>> colormaps_;
  static absl::Mutex colormaps_mutex_;
};

std::map<std::string, std::vector<Color>> ColormapData::colormaps_;
absl::Mutex ColormapData::colormaps_mutex_;

}  // namespace

Colormap::Colormap(const std::string &scheme, double min, double max)
    : data_(GetPredefinedColormapData(scheme)), min_(min), range_(max - min) {
  CHECK_GE(data_.size(), 2);
}

Colormap::Colormap(std::vector<Color> custom_data, double min, double max)
    : custom_data_(std::move(custom_data)),
      data_(custom_data_),
      min_(min),
      range_(max - min) {}

Color Colormap::MapWithDefaultScheme(double normalized_value) {
  return MapWithScheme(normalized_value, kDefaultScheme);
}

Color Colormap::MapWithScheme(double normalized_value,
                              const std::string &scheme) {
  return Colormap(scheme).Map(normalized_value);
}

const std::vector<Color> &Colormap::GetPredefinedColormapData(
    const std::string &scheme) {
  return ColormapData::GetData(scheme);
}

}  // namespace vis
}  // namespace qcraft
