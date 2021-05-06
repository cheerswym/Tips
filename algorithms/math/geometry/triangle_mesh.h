#ifndef ONBOARD_MATH_GEOMETRY_TRIANGLE_MESH_H_
#define ONBOARD_MATH_GEOMETRY_TRIANGLE_MESH_H_

#include <utility>
#include <vector>

#include "onboard/lite/check.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace tmesh {

// A simple triangle mesh class, providing support for vertex/face attached
// data that resizes with the mesh. No support for modifying geometry or
// topology after creation. No suppot for collision detection acceleration such
// as kd-tree or spatial hashing yet.
class TriangleMesh {
 public:
  // Mesh construction.
  int AddVertex(Vec3d v);
  int AddFace(Vec3i f);

  void SetVertices(std::vector<Vec3d> vertices);
  void SetFaces(std::vector<Vec3i> faces);

  // Mesh access.
  int nv() const { return static_cast<int>(vertices_.size()); }
  int nf() const { return static_cast<int>(faces_.size()); }
  const std::vector<Vec3d> &vertices() const { return vertices_; }
  const std::vector<Vec3i> &faces() const { return faces_; }
  const Vec3d &v(int i) const { return vertices_[i]; }
  const Vec3i &f(int i) const { return faces_[i]; }

  // Support for vertex/face attached data that resizes with the mesh
  // automaticaly.
  class ResizeListener {
   public:
    virtual ~ResizeListener() {}
    virtual void Resize(int new_size) = 0;
  };
  void RegisterVertexResizeListener(ResizeListener *listener);
  void UnregisterVertexResizeListener(ResizeListener *listener);
  void RegisterFaceResizeListener(ResizeListener *listener);
  void UnregisterFaceResizeListener(ResizeListener *listener);

 private:
  std::vector<Vec3d> vertices_;
  std::vector<Vec3i> faces_;

  std::vector<ResizeListener *> vertex_resize_listeners_;
  std::vector<ResizeListener *> face_resize_listeners_;
};

// Vertex attached data.
template <typename T>
class VertexData : public TriangleMesh::ResizeListener {
 public:
  explicit VertexData(TriangleMesh *triangle_mesh) : mesh_(triangle_mesh) {
    mesh_->RegisterVertexResizeListener(this);
    Resize(mesh_->nv());
  }
  ~VertexData() { mesh_->UnregisterVertexResizeListener(this); }

  int size() const { return static_cast<int>(data_.size()); }
  const std::vector<T> &data() const { return data_; }
  const T &data(int index) const { return data_[index]; }
  T *mutable_data(int index) { return &data_[index]; }
  void SetData(std::vector<T> data) {
    QCHECK_EQ(data.size(), data_.size());
    data_ = std::move(data);
  }

 protected:
  friend class TriangleMesh;
  void Resize(int new_size) override { data_.resize(new_size); }

 private:
  // Not owned, not null.
  TriangleMesh *mesh_;
  std::vector<T> data_;
};

// Face attached data.
template <typename T>
class FaceData : public TriangleMesh::ResizeListener {
 public:
  explicit FaceData(TriangleMesh *triangle_mesh) : mesh_(triangle_mesh) {
    mesh_->RegisterFaceResizeListener(this);
    Resize(mesh_->nf());
  }
  ~FaceData() { mesh_->UnregisterFaceResizeListener(this); }

  int size() const { return static_cast<int>(data_.size()); }
  const std::vector<T> &data() const { return data_; }
  const T &data(int index) const { return data_[index]; }
  T *mutable_data(int index) { return &data_[index]; }
  void SetData(std::vector<T> data) {
    QCHECK_EQ(data.size(), data_.size());
    data_ = std::move(data);
  }

 protected:
  friend class TriangleMesh;
  void Resize(int new_size) override { data_.resize(new_size); }

 private:
  // Not owned, not null.
  TriangleMesh *mesh_;
  std::vector<T> data_;
};

}  // namespace tmesh
}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_TRIANGLE_MESH_H_
