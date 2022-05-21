#include "onboard/vis/common/renderable_mesh.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "onboard/vis/common/obj_loader.h"
#include "onboard/vis/gl/gl_common.h"

namespace qcraft {
namespace vis {

RenderableMesh::RenderableMesh(const std::string &model_filename,
                               const AffineTransformation &t)
    : normals_(&mesh_) {
  // Load model from filename if format is recognized.
  if (model_filename.size() > 4 &&
      model_filename.substr(model_filename.size() - 4) == ".obj") {
    std::vector<Vec3d> v, vt, vn;
    std::vector<Vec3i> f2v, f2vt, f2vn;
    LoadOBJModelOrDie(model_filename, &v, &vt, &vn, &f2v, &f2vt, &f2vn);

    if (t.mat() != Mat4d::Identity()) {
      for (Vec3d &vertex : v) vertex = t.TransformPoint(vertex);
    }
    mesh_.SetVertices(std::move(v));
    mesh_.SetFaces(std::move(f2v));

    if (!vn.empty()) {
      // Assign the normals to vertices.
      // TODO(Fang) OBJ format allows one vertex to have multiple different
      // normals, depending on which face it is being considered in. This allows
      // for sharp creases in low polygon models. Support it by splitting
      // vertices in those cases (i.e. each distinct v/vt/vn tuple becomes a
      // separate OpenGL vertex).
      for (int i = 0; i < mesh_.nf(); ++i) {
        for (int j = 0; j < 3; ++j) {
          const int v_ix = mesh_.f(i)[j];
          const int vn_ix = f2vn[i][j];
          *normals_.mutable_data(v_ix) = vn[vn_ix];
        }
      }
      // Theoretically there could be vertices not covered by the loop above,
      // but those would be orphan vertices not referenced by any face so they
      // don't matter.
    } else {
      // No normals supplied by the OBJ so we compute them by averaging face
      // normals, weighted by their subtended angles.
      // Reference:
      //   http://brickisland.net/cs177/?p=217
      normals_.SetData(std::vector<Vec3d>(mesh_.nv(), Vec3d::Zero()));
      for (int i = 0; i < mesh_.nf(); ++i) {
        const Vec3i &f = mesh_.f(i);
        const Vec3d fn = (mesh_.v(f.y()) - mesh_.v(f.x()))
                             .cross(mesh_.v(f.z()) - mesh_.v(f.x()))
                             .normalized();
        for (int j = 0; j < 3; ++j) {
          const Vec3d &v0 = mesh_.v(f[j]);
          const Vec3d &v1 = mesh_.v(f[(j + 1) % 3]);
          const Vec3d &v2 = mesh_.v(f[(j + 2) % 3]);
          const double angle = std::acos(std::clamp(
              (v1 - v0).dot(v2 - v0) /
                  std::sqrt((v1 - v0).squaredNorm() * (v2 - v0).squaredNorm()),
              -1.0, 1.0));
          *normals_.mutable_data(f[j]) += angle * fn;
        }
      }
      for (int i = 0; i < mesh_.nv(); ++i) {
        normals_.mutable_data(i)->normalize();
      }
    }
  }

  // Initialize lighting parameters.
  light_dir_ = Vec3d(1.0, 2.0, -1.0).normalized();
  specular_color_ = {1.0, 1.0, 1.0};
  diffuse_color_ = {0.5, 0.5, 0.5};
  ambient_color_ = {0.1, 0.1, 0.1};
}

RenderableMesh::~RenderableMesh() {
  glDeleteBuffers(1, &bo_verts_);
  glDeleteBuffers(1, &bo_faces_);
}

void RenderableMesh::SetLighting(const Vec3d &light_dir,
                                 const Color &specular_color,
                                 const Color &diffuse_color,
                                 const Color &ambient_color) {
  light_dir_ = light_dir.normalized();
  specular_color_ = specular_color;
  diffuse_color_ = diffuse_color;
  ambient_color_ = ambient_color;
}

void RenderableMesh::CheckAndMaybeSetupVBO() {
  if (bo_verts_ > 0 && bo_faces_ > 0) return;

  glGenBuffers(1, &bo_verts_);
  glBindBuffer(GL_ARRAY_BUFFER, bo_verts_);

  // VBO data: interleaved vertex positions and normals.
  double *vbo_data = new double[mesh_.nv() * 6];
  for (int i = 0; i < mesh_.nv(); ++i) {
    vbo_data[i * 6 + 0] = mesh_.v(i).x();
    vbo_data[i * 6 + 1] = mesh_.v(i).y();
    vbo_data[i * 6 + 2] = mesh_.v(i).z();
    vbo_data[i * 6 + 3] = normals_.data(i).x();
    vbo_data[i * 6 + 4] = normals_.data(i).y();
    vbo_data[i * 6 + 5] = normals_.data(i).z();
  }
  glBufferData(GL_ARRAY_BUFFER, mesh_.nv() * sizeof(double) * 6, vbo_data,
               GL_STATIC_DRAW);
  delete[] vbo_data;
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glGenBuffers(1, &bo_faces_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bo_faces_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_.nf() * sizeof(int) * 3,
               mesh_.faces().data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  CheckGLError();
}

void RenderableMesh::Render(const Shader &shader,
                            const AffineTransformation &t) {
  CheckAndMaybeSetupVBO();

  glUseProgram(shader.program());

  const Mat4d model_mat = t.mat();

  Mat4d view_mat;
  glGetDoublev(GL_MODELVIEW_MATRIX, view_mat.data());
  const Vec3d view_dir =
      (view_mat.block<3, 3>(0, 0).inverse() * Vec3d(0.0, 0.0, -1.0))
          .normalized();
  const Vec3d halfway_vec = (view_dir + light_dir_).normalized();

  Mat4d projection_mat;
  glGetDoublev(GL_PROJECTION_MATRIX, projection_mat.data());

  shader.SetUniformMat3d("u_m_it",
                         model_mat.block<3, 3>(0, 0).inverse().transpose());
  shader.SetUniformMat4d("u_mvp", projection_mat * view_mat * model_mat);

  shader.SetUniformVec3d("u_halfway_vec", halfway_vec);
  shader.SetUniformVec3d("u_light_dir", light_dir_);
  shader.SetUniformVec4d("u_specular_color", specular_color_.rgba());
  shader.SetUniformVec4d("u_diffuse_color", diffuse_color_.rgba());
  shader.SetUniformVec4d("u_ambient_color", ambient_color_.rgba());

  glBindBuffer(GL_ARRAY_BUFFER, bo_verts_);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);
  glVertexPointer(3, GL_DOUBLE, 48, reinterpret_cast<GLvoid *>(0));
  glNormalPointer(GL_DOUBLE, 48, reinterpret_cast<GLvoid *>(24));

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bo_faces_);

  glDrawElements(GL_TRIANGLES, mesh_.nf() * 3, GL_UNSIGNED_INT,
                 reinterpret_cast<GLvoid *>(0));

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glUseProgram(0);

  CheckGLError();
}

void RenderableMesh::Render(const AffineTransformation &t) {
  CheckAndMaybeSetupVBO();

  const Mat4d model_mat = t.mat();

  Mat4d view_mat;
  glGetDoublev(GL_MODELVIEW_MATRIX, view_mat.data());
  const Vec3d view_dir =
      (view_mat.block<3, 3>(0, 0).inverse() * Vec3d(0.0, 0.0, -1.0))
          .normalized();
  const Vec3d halfway_vec = (view_dir + light_dir_).normalized();

  const Vec3d specular_rgb = specular_color_.rgb();
  const Vec3d diffuse_rgb = diffuse_color_.rgb();
  const Vec3d ambient_rgb = ambient_color_.rgb();

  Mat4d t_covec_mat = t.mat();
  t_covec_mat.block<3, 3>(0, 0) =
      t_covec_mat.block<3, 3>(0, 0).inverse().transpose();
  AffineTransformation t_covec(t_covec_mat);

  // Per-vertex Blinn-Phong shading.
  constexpr double kPhongSpecularExponent = 250.0;
  glBegin(GL_TRIANGLES);
  for (int i = 0; i < mesh_.nf(); ++i) {
    const Vec3i &f = mesh_.f(i);
    for (int j = 0; j < 3; ++j) {
      const Vec3d x_transformed = t.TransformPoint(mesh_.v(f[j]));
      const Vec3d n_transformed =
          t_covec.TransformVector(normals_.data(f[j])).normalized();
      const double shading_specular =
          std::pow(std::max(0.0, n_transformed.dot(-halfway_vec)),
                   kPhongSpecularExponent);
      const double shading_diffuse =
          (1.0 + n_transformed.dot(-light_dir_)) * 0.5;
      const Vec3d shading = shading_specular * specular_rgb +
                            shading_diffuse * diffuse_rgb + ambient_rgb;
      glColorVec3d(shading);
      glVertexVec3d(x_transformed);
    }
  }
  glEnd();

  CheckGLError();
}

}  // namespace vis
}  // namespace qcraft
