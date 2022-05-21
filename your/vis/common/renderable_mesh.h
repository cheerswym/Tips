#ifndef ONBOARD_VIS_COMMON_RENDERABLE_MESH_H_
#define ONBOARD_VIS_COMMON_RENDERABLE_MESH_H_

#include <string>

#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/geometry/triangle_mesh.h"
#include "onboard/math/vec.h"
#include "onboard/vis/common/color.h"
#include "onboard/vis/gl/shader.h"

namespace qcraft {
namespace vis {

// An untextured triangle mesh for rendering.
class RenderableMesh {
 public:
  explicit RenderableMesh(
      const std::string &model_filename,
      const AffineTransformation &t = AffineTransformation());

  ~RenderableMesh();

  void SetLighting(const Vec3d &light_dir, const Color &specular_color,
                   const Color &diffuse_color,
                   const Color &ambient_color = Color::kWhite);

  // Renders this mesh with a custom shader.
  void Render(const Shader &shader,
              const AffineTransformation &t = AffineTransformation());

  // Renders this mesh without a shader.
  // WARNING: because no shader is given, this function has to perform shading
  // on CPU which is slow. Only use in testing.
  void Render(const AffineTransformation &t = AffineTransformation());

 private:
  // Creates VBO for the mesh.
  void CheckAndMaybeSetupVBO();

  // Triangle soup.
  tmesh::TriangleMesh mesh_;

  // Per-vertex normal vectors.
  tmesh::VertexData<Vec3d> normals_;

  GLuint bo_verts_ = 0;
  GLuint bo_faces_ = 0;

  // Rendering options.
  Vec3d light_dir_;
  Color specular_color_;
  Color diffuse_color_;
  Color ambient_color_;
};

}  // namespace vis
}  // namespace qcraft

#endif  // ONBOARD_VIS_COMMON_RENDERABLE_MESH_H_
