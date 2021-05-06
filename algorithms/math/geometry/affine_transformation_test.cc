#include "onboard/math/geometry/affine_transformation.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

constexpr double kEpsilon = 1e-10;

constexpr double kSqrt2 = 1.4142135623730951;
constexpr double kSqrt2Inv = 1.0 / kSqrt2;
constexpr double kSqrt3 = 1.7320508075688772;

void ExpectNear(const Mat4d &mat0, const Mat4d mat1, double threshold) {
  EXPECT_NEAR(mat0(0, 0), mat1(0, 0), threshold);
  EXPECT_NEAR(mat0(0, 1), mat1(0, 1), threshold);
  EXPECT_NEAR(mat0(0, 2), mat1(0, 2), threshold);
  EXPECT_NEAR(mat0(0, 3), mat1(0, 3), threshold);
  EXPECT_NEAR(mat0(1, 0), mat1(1, 0), threshold);
  EXPECT_NEAR(mat0(1, 1), mat1(1, 1), threshold);
  EXPECT_NEAR(mat0(1, 2), mat1(1, 2), threshold);
  EXPECT_NEAR(mat0(1, 3), mat1(1, 3), threshold);
  EXPECT_NEAR(mat0(2, 0), mat1(2, 0), threshold);
  EXPECT_NEAR(mat0(2, 1), mat1(2, 1), threshold);
  EXPECT_NEAR(mat0(2, 2), mat1(2, 2), threshold);
  EXPECT_NEAR(mat0(2, 3), mat1(2, 3), threshold);
  EXPECT_NEAR(mat0(3, 0), mat1(3, 0), threshold);
  EXPECT_NEAR(mat0(3, 1), mat1(3, 1), threshold);
  EXPECT_NEAR(mat0(3, 2), mat1(3, 2), threshold);
  EXPECT_NEAR(mat0(3, 3), mat1(3, 3), threshold);
}

void ExpectNear(const AffineTransformation &transformation0,
                const AffineTransformation &transformation1, double threshold) {
  ExpectNear(transformation0.mat(), transformation1.mat(), threshold);
}

void ExpectNear(const Mat4d &mat0, const AffineTransformation &transformation1,
                double threshold) {
  ExpectNear(mat0, transformation1.mat(), threshold);
}

[[maybe_unused]] void ExpectNear(const AffineTransformation &transformation0,
                                 const Mat4d &mat1, double threshold) {
  ExpectNear(transformation0.mat(), mat1, threshold);
}

void ExpectNear(double a00, double a01, double a02, double a03, double a10,
                double a11, double a12, double a13, double a20, double a21,
                double a22, double a23, double a30, double a31, double a32,
                double a33, const AffineTransformation &transformation,
                double threshold) {
  Mat4d mat;
  mat << a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31,
      a32, a33;
  ExpectNear(mat, transformation.mat(), threshold);
}

void ExpectNear(const Vec3d x, const Vec3d y, double threshold) {
  EXPECT_NEAR(x.x(), y.x(), threshold);
  EXPECT_NEAR(x.y(), y.y(), threshold);
  EXPECT_NEAR(x.z(), y.z(), threshold);
}

// See affin_transformation.h for the Euler angle convention.
// Reference:
//   http://planning.cs.uiuc.edu/node102.html
void EulerAnglesToRotationMatrix(double yaw, double pitch, double roll,
                                 Mat4d *mat) {
  const double sin_roll = std::sin(roll);
  const double cos_roll = std::cos(roll);
  const double sin_pitch = std::sin(pitch);
  const double cos_pitch = std::cos(pitch);
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);
  (*mat)(0, 0) = cos_yaw * cos_pitch;
  (*mat)(0, 1) = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
  (*mat)(0, 2) = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
  (*mat)(1, 0) = sin_yaw * cos_pitch;
  (*mat)(1, 1) = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
  (*mat)(1, 2) = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
  (*mat)(2, 0) = -sin_pitch;
  (*mat)(2, 1) = cos_pitch * sin_roll;
  (*mat)(2, 2) = cos_pitch * cos_roll;
  (*mat)(0, 3) = 0.0;
  (*mat)(1, 3) = 0.0;
  (*mat)(2, 3) = 0.0;
  (*mat)(3, 0) = 0.0;
  (*mat)(3, 1) = 0.0;
  (*mat)(3, 2) = 0.0;
  (*mat)(3, 3) = 1.0;
}

TEST(AffineTransformation, Set) {
  // Default constructor creates a no-op transformation.
  AffineTransformation t;
  ExpectNear(Mat4d::Identity(), t.mat(), kEpsilon);

  // Test translation.
  t.SetTranslation(3.0, 4.0, 5.0);
  ExpectNear(1.0, 0.0, 0.0, 3.0, 0.0, 1.0, 0.0, 4.0, 0.0, 0.0, 1.0, 5.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);

  // Test rotation around z+ (2D rotation in the x-y plane).
  t.SetRotation(M_PI * 2, 0.0, 0.0, 1.0);
  ExpectNear(Mat4d::Identity(), t.mat(), kEpsilon);
  t.SetRotation(M_PI, 0.0, 0.0, 1.0);
  ExpectNear(-1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);
  t.SetRotation(M_PI / 2, 0.0, 0.0, 1.0);
  ExpectNear(0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);
  t.SetRotation(M_PI / 4, 0.0, 0.0, 1.0);
  ExpectNear(kSqrt2Inv, -kSqrt2Inv, 0.0, 0.0, kSqrt2Inv, kSqrt2Inv, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, t, kEpsilon);

  // Test rotation with an unnormalized vector as axis: the axis will be
  // normalized.
  t.SetRotation(M_PI / 4, 0.0, 0.0, 2.0);
  ExpectNear(kSqrt2Inv, -kSqrt2Inv, 0.0, 0.0, kSqrt2Inv, kSqrt2Inv, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, t, kEpsilon);

  // Test rotation around other axes.
  t.SetRotation(M_PI / 4, 1.0, 0.0, 0.0);
  ExpectNear(1.0, 0.0, 0.0, 0.0, 0.0, kSqrt2Inv, -kSqrt2Inv, 0.0, 0.0,
             kSqrt2Inv, kSqrt2Inv, 0.0, 0.0, 0.0, 0.0, 1.0, t, kEpsilon);
  t.SetRotation(M_PI / 4, 0.0, 1.0, 0.0);
  ExpectNear(kSqrt2Inv, 0.0, kSqrt2Inv, 0.0, 0.0, 1.0, 0.0, 0.0, -kSqrt2Inv,
             0.0, kSqrt2Inv, 0.0, 0.0, 0.0, 0.0, 1.0, t, kEpsilon);
  t.SetRotation(M_PI / 4, 0.0, 0.0, -1.0);
  ExpectNear(kSqrt2Inv, kSqrt2Inv, 0.0, 0.0, -kSqrt2Inv, kSqrt2Inv, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, t, kEpsilon);
  // Reference:
  // https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
  t.SetRotation(M_PI / 4, 1.0, 1.0, 1.0);
  const double coef1 = kSqrt2Inv + (1.0 - kSqrt2Inv) / 3.0;
  const double coef2 = (1.0 - kSqrt2Inv) / 3.0 - kSqrt2Inv / kSqrt3;
  const double coef3 = (1.0 - kSqrt2Inv) / 3.0 + kSqrt2Inv / kSqrt3;
  ExpectNear(coef1, coef2, coef3, 0.0, coef3, coef1, coef2, 0.0, coef2, coef3,
             coef1, 0.0, 0.0, 0.0, 0.0, 1.0, t, kEpsilon);

  // Test scaling.
  t.SetScaling(3.0, 4.0, 5.0);
  ExpectNear(3.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);

  // Test setting from another transformation.
  AffineTransformation t1;
  t1.Set(t);
  ExpectNear(t, t1, kEpsilon);
}

TEST(AffineTransformation, EulerAngles) {
  Vec3d yaw_pitch_roll(0.5, 0.7, 0.9);
  Mat4d mat;
  EulerAnglesToRotationMatrix(yaw_pitch_roll.x(), yaw_pitch_roll.y(),
                              yaw_pitch_roll.z(), &mat);

  AffineTransformation t;
  t.SetYawPitchRoll(yaw_pitch_roll);

  ExpectNear(mat, t, kEpsilon);
}

TEST(AffineTransformation, QuaternionToYawPitchRoll) {
  const Vec3d yaw_pitch_roll(0.0, 0.03, 0.02);
  Mat4d mat;
  EulerAnglesToRotationMatrix(yaw_pitch_roll.x(), yaw_pitch_roll.y(),
                              yaw_pitch_roll.z(), &mat);
  const AffineTransformation t(mat);
  const Vec3d yaw_pitch_roll2 =
      AffineTransformation::QuaternionToYawPitchRoll(t.GetRotation());
  Mat4d mat2;
  EulerAnglesToRotationMatrix(yaw_pitch_roll2.x(), yaw_pitch_roll2.y(),
                              yaw_pitch_roll2.z(), &mat2);
  const AffineTransformation t2(mat2);
  ExpectNear(t, t2, 1e-10);
}

TEST(AffineTransformation, Apply) {
  AffineTransformation t;
  t.ApplyTranslation(3.0, 4.0, 5.0);
  ExpectNear(1.0, 0.0, 0.0, 3.0, 0.0, 1.0, 0.0, 4.0, 0.0, 0.0, 1.0, 5.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);

  t.ApplyTranslation(1.0, 2.0, 3.0);
  ExpectNear(1.0, 0.0, 0.0, 4.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, 1.0, 8.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);

  t.ApplyScaling(3.0, 2.0, 1.0);
  ExpectNear(3.0, 0.0, 0.0, 4.0, 0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 1.0, 8.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);

  t.ApplyTranslation(1.0, 0.0, 0.0);
  ExpectNear(3.0, 0.0, 0.0, 7.0, 0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 1.0, 8.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);

  t.ApplyRotation(M_PI, 0.0, 0.0, 1.0);
  ExpectNear(-3.0, 0.0, 0.0, 7.0, 0.0, -2.0, 0.0, 6.0, 0.0, 0.0, 1.0, 8.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);

  t.ApplyTranslation(0.0, 1.0, 0.0);
  ExpectNear(-3.0, 0.0, 0.0, 7.0, 0.0, -2.0, 0.0, 4.0, 0.0, 0.0, 1.0, 8.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);
}

TEST(AffineTransformation, ProtoConversion) {
  AffineTransformation t;

  // Test translation.
  TranslationProto translation;
  translation.set_x(3.0);
  translation.set_y(4.0);
  translation.set_z(5.0);
  EXPECT_LT((AffineTransformation::TranslationProtoToVec3d(translation) -
             Vec3d(3.0, 4.0, 5.0))
                .squaredNorm(),
            kEpsilon);
  AffineTransformation::Vec3dToTranslationProto(Vec3d(3.0, 4.0, 5.0),
                                                &translation);
  EXPECT_NEAR(translation.x(), 3.0, kEpsilon);
  EXPECT_NEAR(translation.y(), 4.0, kEpsilon);
  EXPECT_NEAR(translation.z(), 5.0, kEpsilon);
  t.Set(translation);
  ExpectNear(AffineTransformation::FromTranslation(3.0, 4.0, 5.0), t, kEpsilon);

  // Test rotation.
  RotationProto rotation;
  rotation.set_angle(90.0);
  rotation.set_ax(1.0);
  rotation.set_ay(0.0);
  rotation.set_az(0.0);
  EXPECT_LT((AffineTransformation::RotationProtoToQuaternion(rotation)
                 .vec()
                 .normalized() -
             Vec3d(1.0, 0.0, 0.0))
                .squaredNorm(),
            kEpsilon);
  AffineTransformation::QuaternionToRotationProto(
      Quaternion(Eigen::AngleAxisd(M_PI / 2, Vec3d(1.0, 0.0, 0.0))), &rotation);
  EXPECT_NEAR(rotation.angle(), 90.0, kEpsilon);
  EXPECT_NEAR(rotation.ax(), 1.0, kEpsilon);
  EXPECT_NEAR(rotation.ay(), 0.0, kEpsilon);
  EXPECT_NEAR(rotation.az(), 0.0, kEpsilon);
  t.Set(rotation);
  ExpectNear(AffineTransformation::FromRotation(M_PI / 2, 1.0, 0.0, 0.0), t,
             kEpsilon);

  // Test scaling.
  ScalingProto scaling;
  scaling.set_sx(3.0);
  scaling.set_sy(4.0);
  scaling.set_sz(5.0);
  EXPECT_LT((AffineTransformation::ScalingProtoToVec3d(scaling) -
             Vec3d(3.0, 4.0, 5.0))
                .squaredNorm(),
            kEpsilon);
  AffineTransformation::Vec3dToScalingProto(Vec3d(3.0, 4.0, 5.0), &scaling);
  EXPECT_NEAR(scaling.sx(), 3.0, kEpsilon);
  EXPECT_NEAR(scaling.sy(), 4.0, kEpsilon);
  EXPECT_NEAR(scaling.sz(), 5.0, kEpsilon);
  t.Set(scaling);
  ExpectNear(AffineTransformation::FromScaling(3.0, 4.0, 5.0), t, kEpsilon);
}

TEST(AffineTransformation, ComponentExtraction) {
  AffineTransformation t;

  // Test recovery of pure translation.
  t.SetTranslation(3.0, 4.0, 5.0);
  Vec3d translation = t.GetTranslation();
  ExpectNear(translation, {3.0, 4.0, 5.0}, kEpsilon);

  // Test recovery of pure rotation.
  t.SetRotation(M_PI / 3, 0.6, 0.8, 0.0);
  Quaternion rotation = t.GetRotation();
  EXPECT_NEAR(rotation.w(), std::cos(M_PI / 3 / 2), kEpsilon);
  EXPECT_NEAR(rotation.x(), std::sin(M_PI / 3 / 2) * 0.6, kEpsilon);
  EXPECT_NEAR(rotation.y(), std::sin(M_PI / 3 / 2) * 0.8, kEpsilon);
  EXPECT_NEAR(rotation.z(), 0.0, kEpsilon);

  // Test recovery of pure scaling.
  t.SetScaling(3.0, 4.0, 5.0);
  Vec3d scaling = t.GetScaling();
  ExpectNear(scaling, {3.0, 4.0, 5.0}, kEpsilon);

  t.SetScaling(5.0, 3.0, 4.0);
  scaling = t.GetScaling();
  ExpectNear(scaling, {5.0, 3.0, 4.0}, kEpsilon);

  t.SetScaling(3.0, 4.0, -5.0);
  scaling = t.GetScaling();
  ExpectNear(scaling, {3.0, 4.0, -5.0}, kEpsilon);

  t.SetScaling(-3.0, 4.0, -5.0);
  scaling = t.GetScaling();
  ExpectNear(scaling, {-3.0, 4.0, -5.0}, kEpsilon);

  // Test decomposition of a compound transformation.
  t.SetTranslation(3.0, 4.0, 5.0);
  t.ApplyRotation(M_PI / 2, 0.0, 0.0, 1.0);
  t.ApplyScaling(2.0, 2.0, 2.0);
  t.ApplyTranslation(1.0, 0.0, 2.0);
  t.ApplyRotation(M_PI / 2, 0.0, 1.0, 0.0);
  t.ApplyTranslation(0.0, 0.0, 1.0);
  ExpectNear(0.0, -2.0, 0.0, 3.0, 0.0, 0.0, 2.0, 8.0, -2.0, 0.0, 0.0, 9.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);
  translation = t.GetTranslation();
  ExpectNear(translation, {3.0, 8.0, 9.0}, kEpsilon);
  rotation = t.GetRotation();
  ExpectNear(Vec3d(rotation.vec().normalized()), Vec3d(-1.0, 1.0, 1.0) / kSqrt3,
             kEpsilon);
  EXPECT_NEAR(rotation.w(), std::cos(M_PI / 3), kEpsilon);
  scaling = t.GetScaling();
  ExpectNear(scaling, {2.0, 2.0, 2.0}, kEpsilon);
}

TEST(AffineTransformation, TransformVector) {
  AffineTransformation t;
  const Vec3d p(1.0, 1.0, 1.0);
  const Vec3d v(1.0, 1.0, 1.0);
  const Vec3d n(1.0, 1.0, 1.0);

  t.SetTranslation(3.0, 4.0, 5.0);
  ExpectNear(t.TransformPoint(p), {4.0, 5.0, 6.0}, kEpsilon);
  ExpectNear(t.TransformVector(v), {1.0, 1.0, 1.0}, kEpsilon);
  ExpectNear(t.TransformCovector(n), {1.0, 1.0, 1.0}, kEpsilon);

  t.SetRotation(M_PI / 2, 0.0, 0.0, 1.0);
  ExpectNear(t.TransformPoint(p), {-1.0, 1.0, 1.0}, kEpsilon);
  ExpectNear(t.TransformVector(v), {-1.0, 1.0, 1.0}, kEpsilon);
  ExpectNear(t.TransformCovector(n), {-1.0, 1.0, 1.0}, kEpsilon);

  t.SetScaling(2.0, 3.0, 4.0);
  ExpectNear(t.TransformPoint(p), {2.0, 3.0, 4.0}, kEpsilon);
  ExpectNear(t.TransformVector(v), {2.0, 3.0, 4.0}, kEpsilon);
  ExpectNear(t.TransformCovector(n), {0.5, 0.333333333333333, 0.25}, kEpsilon);
}

TEST(AffineTransformation, TransformationSequence) {
  AffineTransformation t;
  AffineTransformationSequenceProto sequence;

  // Same sequence of operations as the last test in ComponentExtraction.
  {
    auto *translation = sequence.add_operations()->mutable_translation();
    translation->set_x(3.0);
    translation->set_y(4.0);
    translation->set_z(5.0);
  }
  {
    auto *rotation = sequence.add_operations()->mutable_rotation();
    rotation->set_angle(90.0);
    rotation->set_ax(0.0);
    rotation->set_ay(0.0);
    rotation->set_az(1.0);
  }
  {
    auto *scaling = sequence.add_operations()->mutable_scaling();
    scaling->set_sx(2.0);
    scaling->set_sy(2.0);
    scaling->set_sz(2.0);
  }
  {
    auto *translation = sequence.add_operations()->mutable_translation();
    translation->set_x(1.0);
    translation->set_y(0.0);
    translation->set_z(2.0);
  }
  {
    auto *rotation = sequence.add_operations()->mutable_rotation();
    rotation->set_angle(90.0);
    rotation->set_ax(0.0);
    rotation->set_ay(1.0);
    rotation->set_az(0.0);
  }
  {
    auto *translation = sequence.add_operations()->mutable_translation();
    translation->set_x(0.0);
    translation->set_y(0.0);
    translation->set_z(1.0);
  }
  t.Set(sequence);
  ExpectNear(0.0, -2.0, 0.0, 3.0, 0.0, 0.0, 2.0, 8.0, -2.0, 0.0, 0.0, 9.0, 0.0,
             0.0, 0.0, 1.0, t, kEpsilon);
}

TEST(AffineTransformation, SmallRotationYawPitchRoll) {
  for (const auto &axis : {Vec3d::UnitX(), Vec3d::UnitY(), Vec3d::UnitZ()}) {
    for (const double angle : {1.0, -1.0}) {  // Degrees.
      for (const double dx : {0.01, 0.0, -0.01}) {
        for (const double dy : {0.01, 0.0, -0.01}) {
          RotationProto rotation;
          rotation.set_angle(angle);
          rotation.set_ax(axis.x());
          rotation.set_ay(axis.y() + dx);
          rotation.set_az(axis.z() + dy);
          const Vec3d yaw_pitch_roll =
              AffineTransformation(rotation).GetRotationYawPitchRoll();
          //        std::cout << axis.transpose() << "; " << angle << "; " << dx
          //        << ", "
          //                  << dy << ": " << yaw_pitch_roll.transpose() <<
          //                  std::endl;
          EXPECT_NEAR(yaw_pitch_roll.norm(), 0.0, 0.1);
        }
      }
    }
  }
}

}  // namespace
}  // namespace qcraft

// clang-format on
