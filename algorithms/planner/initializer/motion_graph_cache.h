#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_H_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_graph.h"

namespace qcraft::planner {
// motion graph cache is to collect the motion edges calculated as well as their
// related costs (except for the leading object costs);

// (v, acc, t, GeometryForm) -> costs (or summation oftthe calcualted costs.)

// For one particular node, the motion form ended at this node should be
// specified by: acc, init_v, and the GeometryForm (which is specified by
// start_node, end_node, as well as the type of the connection: lateral quintic
// polynomials or quintic spirals.)

struct MotionEdgeCache {
  std::unique_ptr<MotionForm> ptr_motion_form;
  std::vector<double> costs;
};

// Discrete motion edge sample
struct MotionEdgeKey {
  // One specific motion edge sample can be uniquely defined as a geometry edge
  // (defined as the start node, end node as well as the geometry edge type)
  // plus it's initial velocity and acceleration. Then a MotionEdgeKey can be
  // created using DpMotionInfo directly during motion search.

  int acc;
  int init_v;
  int t;
  GeometryEdgeIndex geom_edge_index;

  double v0() const { return init_v / 100.0; }
  double a0() const { return acc / 100.0; }
  double t0() const { return t / 100.0; }
  std::string DebugString() const {
    return absl::StrCat(acc, "\t", init_v, "\t", t, "\t",
                        geom_edge_index.value());
  }

  MotionEdgeKey(double acc, double init_v, double t,
                GeometryEdgeIndex geom_edge_index)
      : acc(round(acc * 100)),
        init_v(round(init_v * 100)),
        t(round(t * 100)),
        geom_edge_index(geom_edge_index) {}

  MotionEdgeKey()
      : acc(0),
        init_v(0),
        t(0),
        geom_edge_index(GeometryEdgeVector<GeometryEdge>::kInvalidIndex) {}

  friend bool operator==(const MotionEdgeKey &lhs, const MotionEdgeKey &rhs) {
    return lhs.acc == rhs.acc && lhs.t == rhs.t && lhs.init_v == rhs.init_v &&
           lhs.geom_edge_index.value() == rhs.geom_edge_index.value();
  }

  template <typename H>
  friend H AbslHashValue(H h, const MotionEdgeKey &mek) {
    return H::combine(std::move(h), mek.acc, mek.init_v, mek.t,
                      mek.geom_edge_index.value());
  }
};

struct NewCacheInfo {
  MotionEdgeKey key;
  MotionEdgeCache cache;
};

// Main structure to hold information during dynamic programming.
struct DpMotionInfo {
  MotionEdgeKey key;
  double sum_cost = 0.0;
  std::vector<double> costs;
  double start_t = 0.0;
  MotionEdgeIndex prev_motion_edge_index;
  GeometryNodeIndex end_geometry_node_index;
  MotionForm *motion_form;
  GeometryEdgeIndex geometry_edge_index;

  std::string DebugString() const {
    return absl::StrCat(
        "sum_cost: ", sum_cost, "start_t: ", start_t,
        "prev_motion_edge_index: ", prev_motion_edge_index.value(),
        "end_geometry_node_index: ", end_geometry_node_index.value(),
        "\n Geometry edge index: ", geometry_edge_index.value());
  }
};

class MotionGraphCache {
 public:
  explicit MotionGraphCache(const GeometryGraph *geom_graph);

  void BatchGetOrFail(const std::vector<MotionEdgeKey> &samples,
                      std::vector<DpMotionInfo> *ptr_result,
                      std::vector<int> *failed_idx) const;

  void Insert(const MotionEdgeKey &key, const std::vector<double> &costs,
              std::unique_ptr<MotionForm> ptr_motion_form);

  void BatchInsert(std::vector<NewCacheInfo> new_motion_forms);

  inline bool has(const MotionEdgeKey &key) const {
    return cache_.contains(key);
  }

  absl::StatusOr<MotionForm *> GetMotionForm(const MotionEdgeKey &key) const;

  absl::StatusOr<std::vector<double>> GetCosts(const MotionEdgeKey &key) const;

  inline int size() const { return cache_.size(); }

  ~MotionGraphCache() { MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(cache_); }

 private:
  [[maybe_unused]] const GeometryGraph *geom_graph_;
  absl::flat_hash_map<MotionEdgeKey, MotionEdgeCache> cache_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_
