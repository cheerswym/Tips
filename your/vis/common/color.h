#ifndef ONBOARD_VIS_COMMON_COLOR_H_
#define ONBOARD_VIS_COMMON_COLOR_H_

#include <string>

#include "absl/strings/str_format.h"
#include "onboard/math/eigen.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/vis/common/proto/color.pb.h"

namespace qcraft {
namespace vis {

class Color {
 public:
  // Commonly used colors.
  static const Color kTransparent;
  static const Color kBlack;
  static const Color kSmokyBlack;
  static const Color kDarkGray;
  static const Color kGray;
  static const Color kLightGray;
  static const Color kWhite;
  static const Color kRed;
  static const Color kTomato;
  static const Color kCrimson;
  static const Color kGreen;
  static const Color kOliveDrab;
  static const Color kBlue;
  static const Color kYellow;
  static const Color kGold;
  static const Color kCyan;
  static const Color kMagenta;
  static const Color kOrange;
  static const Color kSandyBrown;
  static const Color kBrown;
  static const Color kBronze;
  static const Color kChocolate;
  static const Color kMint;
  static const Color kIndigo;
  static const Color kViolet;
  static const Color kSalmonPink;
  static const Color kMediumPurple;
  static const Color kSkyBlue;
  static const Color kLightRed;
  static const Color kLightGreen;
  static const Color kLimeGreen;
  static const Color kLightBlue;
  static const Color kLightYellow;
  static const Color kLightCyan;
  static const Color kTiffanyBlue;
  static const Color kLightMagenta;
  static const Color kLightOrange;
  static const Color kLightPurple;
  static const Color kLightSkyBlue;
  static const Color kMediumBlue;
  static const Color kAzure;
  static const Color kMiddleBlueGreen;
  static const Color kDarkRed;
  static const Color kIslamicGreen;
  static const Color kSeaGreen;
  static const Color kDarkGreen;
  static const Color kHanPurple;
  static const Color kDarkBlue;
  static const Color kDarkYellow;
  static const Color kDarkCyan;
  static const Color kDarkMagenta;
  static const Color kDarkViolet;

  static Color Random(const double alpha = 1.0);

  // Combining rgba ints into one int32
  static int32_t Int32FromRGBA(int r, int g, int b, int a) {
    return (r << 24) + (g << 16) + (b << 8) + (a << 0);
  }
  static int32_t Int32FromRGB(int r, int g, int b) {
    return Int32FromRGBA(r, g, b, 0);
  }

  // Setting a color without the alpha channel is equivalent to alpha = 1.0.
  Color() = default;
  Color(double r, double g, double b) { Set(r, g, b); }
  Color(double r, double g, double b, double a) { Set(r, g, b, a); }
  explicit Color(const Vec3d &c) { Set(c); }
  explicit Color(const Vec4d &c) { Set(c); }
  explicit Color(const ColorRGBAProto &proto) { Set(proto); }
  explicit Color(int32_t rgba) { Set(rgba); }

  void Set(double r, double g, double b) { c_ = Vec4d(r, g, b, 1.0); }
  void Set(double r, double g, double b, double a) { c_ = Vec4d(r, g, b, a); }
  void Set(const Vec3d &c) { c_ = Vec4d(c.x(), c.y(), c.z(), 1.0); }
  void Set(const Vec4d &c) { c_ = c; }
  void Set(const ColorRGBAProto &proto) { FromProto(proto); }
  void Set(int32_t rgba) { FromInt32(rgba); }

  double r() const { return c_.x(); }
  double g() const { return c_.y(); }
  double b() const { return c_.z(); }
  double a() const { return c_.w(); }
  const Vec4d &rgba() const { return c_; }
  Vec3d rgb() const { return Vec3d(r(), g(), b()); }

  double &r() { return c_.x(); }
  double &g() { return c_.y(); }
  double &b() { return c_.z(); }
  double &a() { return c_.w(); }

  std::string DebugString() const {
    return absl::StrFormat("{%f, %f, %f, %f}", r(), g(), b(), a());
  }

  std::string DebugStringWebStyle() const {
    return absl::StrFormat("#%0X", ToInt32());
  }

  // Serialization to int32. Note that the quantization scheme below ensures
  // that repeated back-and-forth conversion is stable, but this inevitably
  // results in loss of certain desired extreme values (e.g. quantizing 1.0
  // yields 255, while dequantizing 255 yields 0.998 which is mean of the values
  // to quantize to 255).
  int ToInt32() const {
    const int byte_r = std::clamp(FloorToInt(r() * 0x100), 0, 0xff);
    const int byte_g = std::clamp(FloorToInt(g() * 0x100), 0, 0xff);
    const int byte_b = std::clamp(FloorToInt(b() * 0x100), 0, 0xff);
    const int byte_a = std::clamp(FloorToInt(a() * 0x100), 0, 0xff);
    return (byte_r << 24) + (byte_g << 16) + (byte_b << 8) + (byte_a << 0);
  }
  void FromInt32(int32_t rgba) {
    const double float_r = (((rgba & 0xff000000) >> 24) + 0.5) / 256.0;
    const double float_g = (((rgba & 0xff0000) >> 16) + 0.5) / 256.0;
    const double float_b = (((rgba & 0xff00) >> 8) + 0.5) / 256.0;
    const double float_a = ((rgba & 0xff) + 0.5) / 256.0;
    Set(float_r, float_g, float_b, float_a);
  }

  // Serialization to protobuf. This uses serialization to int32 as the
  // implementation.
  void ToProto(ColorRGBAProto *proto) const { proto->set_rgba(ToInt32()); }
  void FromProto(const ColorRGBAProto &proto) { FromInt32(proto.rgba()); }

  // Conversion with HSV space. H, S and V are all normalized (from 0 to 1).
  Vec3d ToHsv() const;
  void FromHsv(Vec3d hsv);
  Color WithAlpha(const double alpha) const {
    return {c_.x(), c_.y(), c_.z(), alpha};
  }

 private:
  Vec4d c_;
};

}  // namespace vis
}  // namespace qcraft

#endif  // ONBOARD_VIS_COMMON_COLOR_H_
