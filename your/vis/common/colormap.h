#ifndef ONBOARD_VIS_COMMON_COLORMAP_H_
#define ONBOARD_VIS_COMMON_COLORMAP_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "onboard/vis/common/color.h"

namespace qcraft {
namespace vis {

class Colormap {
 public:
  // Commonly used colormap schemes: "jet", "magma", "viridis", "seismic",
  // "rainbow"
  //
  // List of all colormap schemes supported by this class:
  //   https://docs.google.com/document/d/1PBu0FpbcLcZQHeSS88CINC3rWyEMAs7FmD7ns4sfdEg/edit?usp=sharing
  //
  // List of Matplotlib colormaps:
  //   https://matplotlib.org/tutorials/colors/colormaps.html
  // List of Matlab colormaps:
  //   https://www.mathworks.com/help/matlab/ref/colormap.html
  //
  static constexpr char kDefaultScheme[] = "jet";
  explicit Colormap(const std::string &scheme = kDefaultScheme,
                    double min = 0.0, double max = 1.0);
  explicit Colormap(std::vector<Color> custom_data, double min = 0.0,
                    double max = 1.0);

  Color Map(double value) const;
  Color operator()(double value) const { return Map(value); }

  static Color MapWithDefaultScheme(double normalized_value);
  static Color MapWithScheme(double normalized_value,
                             const std::string &scheme);

 protected:
  static const std::vector<Color> &GetPredefinedColormapData(
      const std::string &scheme);

 private:
  const std::vector<Color> custom_data_;
  const std::vector<Color> &data_;
  double min_ = 0.0;
  double range_ = 1.0;
};

inline Color Colormap::Map(double value) const {
  const double normalized_value = std::clamp((value - min_) / range_, 0.0, 1.0);

  const double mapped_value = normalized_value * (data_.size() - 1);
  int index = FloorToInt(mapped_value);
  if (index == data_.size() - 1) {
    index = data_.size() - 2;
  }
  DCHECK_GE(index, 0);
  DCHECK_LE(index, data_.size() - 2);

  const double frac = mapped_value - index;
  DCHECK_GE(frac, 0.0);
  DCHECK_LE(frac, 1.0);

  Color color0 = data_[index];
  Color color1 = data_[index + 1];
  return Color(Vec4d(color0.rgba() * (1.0 - frac) + color1.rgba() * frac));
}

}  // namespace vis
}  // namespace qcraft

#endif  // ONBOARD_VIS_COMMON_COLORMAP_H_
