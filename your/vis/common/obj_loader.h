#ifndef ONBOARD_VIS_COMMON_OBJ_LOADER_H_
#define ONBOARD_VIS_COMMON_OBJ_LOADER_H_

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace vis {

// An OBJ file parser, loading the following lines:
//   v: vertices
//   vt: vertex texture coordinates
//   vn: vertex normals
//   f: faces
// The loader ignores material info. It also breaks face polygons into
// triangles, and offsets the vertex indices so that they are 0-based rather
// than 1-based as is the native convention of OBJ.
template <typename Vec3>
void LoadOBJModelOrDie(const std::string &filename, std::vector<Vec3> *v,
                       std::vector<Vec3> *vt, std::vector<Vec3> *vn,
                       std::vector<Vec3i> *f2v, std::vector<Vec3i> *f2vt,
                       std::vector<Vec3i> *f2vn) {
  CHECK(v != nullptr);
  CHECK(vt != nullptr);
  CHECK(vn != nullptr);
  CHECK(f2v != nullptr);
  CHECK(f2vt != nullptr);
  CHECK(f2vn != nullptr);

  std::ifstream fin(filename, std::ios::in);
  CHECK(fin.is_open());
  LOG(INFO) << "Loading OBJ file " << filename;

  v->clear();
  vt->clear();
  vn->clear();
  f2v->clear();
  f2vt->clear();
  f2vn->clear();
  Vec3 bb_min = Vec3::Ones() * std::numeric_limits<double>::max();
  Vec3 bb_max = -Vec3::Ones() * std::numeric_limits<double>::max();
  while (!fin.eof()) {
    std::string line;
    std::getline(fin, line);
    std::stringstream ss(line);
    std::string command;
    ss >> command;
    if (command == "v" || command == "vt" || command == "vn") {
      // Read vertex coordinates.
      double x, y, z;
      ss >> x >> y >> z;
      const Vec3 vec(x, y, z);
      (command == "v" ? v : (command == "vt" ? vt : vn))->push_back(vec);
      if (command == "v") {
        bb_min.x() = std::min(bb_min.x(), vec.x());
        bb_min.y() = std::min(bb_min.y(), vec.y());
        bb_min.z() = std::min(bb_min.z(), vec.z());
        bb_max.x() = std::max(bb_max.x(), vec.x());
        bb_max.y() = std::max(bb_max.y(), vec.y());
        bb_max.z() = std::max(bb_max.z(), vec.z());
      }
    } else if (command == "f") {
      // Read face polygon (vert/texcoord/normal) and triangulage it.
      std::vector<Vec3i> polygon;
      while (!ss.eof()) {
        std::string s;
        ss >> s;
        if (s != "") {
          // Split by '/'.
          int int1 = 0, int2 = 0;
          for (int i = 0; i < s.size(); ++i) {
            if (s[i] == '/') {
              if (int1 == 0) {
                int1 = i;
              } else {
                int2 = i;
                break;
              }
            }
          }
          // A valid OBJ file must have either one or three indices in each
          // vertex of a face.
          char *_;
          if (int1 != 0) {
            DCHECK_NE(int2, 0) << "The OBJ file is ill-formed: a face vertex \""
                               << s << "\" has two indices.";
            polygon.emplace_back(std::strtol(s.c_str(), &_, 10),
                                 std::strtol(s.c_str() + int1 + 1, &_, 10),
                                 std::strtol(s.c_str() + int2 + 1, &_, 10));
          } else {
            polygon.emplace_back(std::strtol(s.c_str(), &_, 10), 0, 0);
          }
        }
      }
      for (int i = 2; i < polygon.size(); ++i) {
        f2v->emplace_back(polygon[0].x() - 1, polygon[i - 1].x() - 1,
                          polygon[i].x() - 1);
        f2vt->emplace_back(polygon[0].y() - 1, polygon[i - 1].y() - 1,
                           polygon[i].y() - 1);
        f2vn->emplace_back(polygon[0].z() - 1, polygon[i - 1].z() - 1,
                           polygon[i].z() - 1);
      }
    }
  }
  fin.close();
  VLOG(1) << "Loaded OBJ model " << filename << ": " << v->size()
          << " vertices, " << vt->size() << " texture coordinates, "
          << vn->size() << " normals, " << f2v->size() << " faces.";
  VLOG(1) << "AABB: (" << bb_min.transpose() << ") - (" << bb_max.transpose()
          << ")" << std::endl;

  // Sanity check.
  const int nv = v->size();
  const int nvt = vt->size();
  const int nvn = vn->size();
  for (const Vec3i &f : *f2v) {
    DCHECK_LT(f.x(), nv);
    DCHECK_LT(f.y(), nv);
    DCHECK_LT(f.z(), nv);
    DCHECK_GE(f.x(), 0);
    DCHECK_GE(f.y(), 0);
    DCHECK_GE(f.z(), 0);
  }
  for (const Vec3i &f : *f2vt) {
    DCHECK_LT(f.x(), nvt);
    DCHECK_LT(f.y(), nvt);
    DCHECK_LT(f.z(), nvt);
    if (f.x() < 0 || f.y() < 0 || f.z() < 0) DCHECK_EQ(nvt, 0);
  }
  for (const Vec3i &f : *f2vn) {
    DCHECK_LT(f.x(), nvn);
    DCHECK_LT(f.y(), nvn);
    DCHECK_LT(f.z(), nvn);
    if (f.x() < 0 || f.y() < 0 || f.z() < 0) DCHECK_EQ(nvn, 0);
  }
}

}  // namespace vis
}  // namespace qcraft

#endif  // ONBOARD_VIS_COMMON_OBJ_LOADER_H_
