#include "onboard/math/geometry/triangle_mesh.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace tmesh {
namespace {

TEST(TriangleMesh, Creation) {
  TriangleMesh mesh;
  const int v0 = mesh.AddVertex({0.0, 0.0, 0.0});
  const int v1 = mesh.AddVertex({1.0, 0.0, 0.0});
  const int v2 = mesh.AddVertex({1.0, 1.0, 0.0});
  const int v3 = mesh.AddVertex({0.0, 1.0, 0.0});
  EXPECT_EQ(v0, 0);
  EXPECT_EQ(v1, 1);
  EXPECT_EQ(v2, 2);
  EXPECT_EQ(v3, 3);
  EXPECT_EQ(mesh.nv(), 4);
  const int f0 = mesh.AddFace({v0, v1, v2});
  const int f1 = mesh.AddFace({v2, v3, v0});
  EXPECT_EQ(f0, 0);
  EXPECT_EQ(f1, 1);
  ASSERT_EQ(mesh.nf(), 2);
  EXPECT_EQ(mesh.faces()[0], Vec3i(v0, v1, v2));
  EXPECT_EQ(mesh.faces()[1], Vec3i(v2, v3, v0));
}

TEST(TriangleMesh, AttachedData) {
  TriangleMesh mesh;
  const int v0 = mesh.AddVertex({0.0, 0.0, 0.0});
  const int v1 = mesh.AddVertex({1.0, 0.0, 0.0});
  const int v2 = mesh.AddVertex({1.0, 1.0, 0.0});
  mesh.AddFace({v0, v1, v2});
  VertexData<int> vd(&mesh);
  EXPECT_EQ(vd.size(), 3);
  FaceData<int> fd(&mesh);
  EXPECT_EQ(fd.size(), 1);

  const int v3 = mesh.AddVertex({0.0, 1.0, 0.0});
  mesh.AddFace({v2, v3, v0});
  EXPECT_EQ(vd.size(), 4);
  EXPECT_EQ(fd.size(), 2);
}

}  // namespace
}  // namespace tmesh
}  // namespace qcraft
