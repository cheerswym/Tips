#include "onboard/vis/common/color.h"

#include <algorithm>
#include <random>

namespace qcraft {
namespace vis {

// - To visualize a color, navigate to:
// https://doc.instantreality.org/tools/color_calculator/
// - To get a suitable color name from hex code, navigate to:
// https://www.colorabout.com/color/hex/ffffff
// Note the last element in the URL, you can drop in any valid hex color code
// there and see a nearest color name
const Color Color::kTransparent(0.0, 0.0, 0.0, 0.0);
const Color Color::kBlack(0.0, 0.0, 0.0);
const Color Color::kSmokyBlack(0.05, 0.05, 0.05);
const Color Color::kDarkGray(0.25, 0.25, 0.25);
const Color Color::kGray(0.5, 0.5, 0.5);
const Color Color::kLightGray(0.75, 0.75, 0.75);
const Color Color::kWhite(1.0, 1.0, 1.0);
const Color Color::kRed(1.0, 0.0, 0.0);
const Color Color::kTomato(1.0, 0.3, 0.3);
const Color Color::kCrimson(1.0, 0.2, 0.2);
const Color Color::kGreen(0.0, 1.0, 0.0);
const Color Color::kOliveDrab(0.4, 0.7, 0.1);
const Color Color::kBlue(0.0, 0.0, 1.0);
const Color Color::kYellow(1.0, 1.0, 0.0);
const Color Color::kGold(1.0, 0.8, 0.0);
const Color Color::kCyan(0.0, 1.0, 1.0);
const Color Color::kMagenta(1.0, 0.0, 1.0);
const Color Color::kOrange(1.0, 0.5, 0.0);
const Color Color::kSandyBrown(1.0, 0.7, 0.4);
const Color Color::kBrown(0.65, 0.16, 0.16);
const Color Color::kBronze(0.7, 0.4, 0.0);
const Color Color::kChocolate(0.823, 0.411, 0.117);
const Color Color::kMint(0.67, 1.0, 0.76);
const Color Color::kIndigo(0.29, 0, 0.51);
const Color Color::kViolet(0.93, 0.51, 0.93);
const Color Color::kSalmonPink(1.0, 0.6, 0.6);
const Color Color::kMediumPurple(0.6, 0.5, 1.0);
const Color Color::kSkyBlue(0.53, 0.81, 0.92);
const Color Color::kLightRed(1.0, 0.5, 0.5);
const Color Color::kLightGreen(0.5, 1.0, 0.5);
const Color Color::kLimeGreen(0.1, 0.9, 0.1);
const Color Color::kLightBlue(0.5, 0.5, 1.0);
const Color Color::kLightYellow(1.0, 1.0, 0.5);
const Color Color::kLightCyan(0.5, 1.0, 1.0);
const Color Color::kTiffanyBlue(0.039, 0.729, 0.729);
const Color Color::kLightMagenta(1.0, 0.5, 1.0);
const Color Color::kLightOrange(1.0, 0.85, 0.69);
const Color Color::kLightPurple(0.86, 0.75, 1.0);
const Color Color::kLightSkyBlue(0.53, 0.81, 0.98);
const Color Color::kMediumBlue(0.0, 0.2, 0.8);
const Color Color::kAzure(0.0, 0.5, 1.0);
const Color Color::kMiddleBlueGreen(0.5, 0.8, 0.9);
const Color Color::kDarkRed(0.5, 0.0, 0.0);
const Color Color::kIslamicGreen(0.0, 0.7, 0.0);
const Color Color::kSeaGreen(0.2, 0.6, 0.3, 0.0);
const Color Color::kDarkGreen(0.0, 0.5, 0.0);
const Color Color::kHanPurple(0.2, 0.2, 1.0);
const Color Color::kDarkBlue(0.0, 0.0, 0.5);
const Color Color::kDarkYellow(0.5, 0.5, 0.0);
const Color Color::kDarkCyan(0.0, 0.5, 0.5);
const Color Color::kDarkMagenta(0.5, 0.0, 0.5);
const Color Color::kDarkViolet(0.58, 0.0, 0.83);

Color Color::Random(const double alpha) {
  static std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  return {dis(gen), dis(gen), dis(gen), alpha};
}

Vec3d Color::ToHsv() const {
  const double r = this->r();
  const double g = this->g();
  const double b = this->b();
  const double max = std::max(std::max(r, g), b);
  const double min = std::min(std::min(r, g), b);
  const double range = max - min;
  const double value = max;
  const double saturation = max == 0.0 ? 0.0 : range / max;
  double hue = 0.0;
  if (range != 0.0) {
    if (max == r) {
      hue = 1.0 / 6.0 * (g - b) / range;
    } else if (max == g) {
      hue = 1.0 / 6.0 * ((b - r) / range + 2);
    } else {
      hue = 1.0 / 6.0 * ((r - g) / range + 4);
    }
  }
  hue = WrapToRange(hue, 0.0, 1.0);
  return {hue, saturation, value};
}

void Color::FromHsv(Vec3d hsv) {
  const double hue = hsv.x();
  const double saturation = hsv.y();
  const double value = hsv.z();
  const double max = value;
  const double range = max * saturation;
  const double min = max - range;
  const auto fmod = [](double a, double b) {
    while (a >= b) a -= b;
    return a;
  };
  const double secondary = range * (1.0 - std::abs(fmod(hue * 6.0, 2.0) - 1.0));
  double r, g, b;
  if (hue < 1.0 / 6.0) {
    r = range;
    g = secondary;
    b = 0.0;
  } else if (hue < 2.0 / 6.0) {
    r = secondary;
    g = range;
    b = 0.0;
  } else if (hue < 3.0 / 6.0) {
    r = 0.0;
    g = range;
    b = secondary;
  } else if (hue < 4.0 / 6.0) {
    r = 0.0;
    g = secondary;
    b = range;
  } else if (hue < 5.0 / 6.0) {
    r = secondary;
    g = 0.0;
    b = range;
  } else {
    r = range;
    g = 0.0;
    b = secondary;
  }
  Set(r + min, g + min, b + min);
}

}  // namespace vis
}  // namespace qcraft
