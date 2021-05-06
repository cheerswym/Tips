#include "onboard/math/geometry/triangle_mesh.h"

namespace qcraft {
namespace tmesh {

int TriangleMesh::AddVertex(Vec3d v) {
  const int index = static_cast<int>(vertices_.size());
  vertices_.push_back(v);
  for (auto *listener : vertex_resize_listeners_) {
    listener->Resize(nv());
  }
  return index;
}

int TriangleMesh::AddFace(Vec3i f) {
  const int index = static_cast<int>(faces_.size());
  faces_.push_back(f);
  for (auto *listener : face_resize_listeners_) {
    listener->Resize(nf());
  }
  return index;
}

void TriangleMesh::SetVertices(std::vector<Vec3d> vertices) {
  vertices_ = std::move(vertices);
  for (auto *listener : vertex_resize_listeners_) {
    listener->Resize(nv());
  }
}

void TriangleMesh::SetFaces(std::vector<Vec3i> faces) {
  faces_ = std::move(faces);
  for (auto *listener : face_resize_listeners_) {
    listener->Resize(nf());
  }
}

void TriangleMesh::RegisterVertexResizeListener(ResizeListener *listener) {
  vertex_resize_listeners_.push_back(listener);
}

void TriangleMesh::UnregisterVertexResizeListener(ResizeListener *listener) {
  vertex_resize_listeners_.erase(
      std::remove(vertex_resize_listeners_.begin(),
                  vertex_resize_listeners_.end(), listener),
      vertex_resize_listeners_.end());
}

void TriangleMesh::RegisterFaceResizeListener(ResizeListener *listener) {
  face_resize_listeners_.push_back(listener);
}

void TriangleMesh::UnregisterFaceResizeListener(ResizeListener *listener) {
  face_resize_listeners_.erase(
      std::remove(face_resize_listeners_.begin(), face_resize_listeners_.end(),
                  listener),
      face_resize_listeners_.end());
}

}  // namespace tmesh
}  // namespace qcraft
