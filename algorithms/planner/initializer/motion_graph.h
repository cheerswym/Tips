#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_H_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "onboard/container/strong_vector.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

constexpr int kDiscreteTimeSampleStep = 1.0;  // seconds.
const int kDiscreteTimeHorizon =
    CeilToInt(kTrajectoryTimeHorizon / kDiscreteTimeSampleStep);  // steps

DECLARE_STRONG_VECTOR(MotionNode);
DECLARE_STRONG_VECTOR(MotionEdge);

struct MotionNode {
  MotionNodeIndex index;
  MotionState state;
  GeometryNodeIndex geom_index;
};

struct MotionEdge {
  MotionNodeIndex start;
  MotionNodeIndex end;
  // TODO(weijun): Delete end_geom
  GeometryNodeIndex end_geom;
  const MotionForm* motion;
  MotionEdgeIndex prev_edge;
};

// Discrete motion samples on geometry node
struct MotionSample {
  int v_discrete;  // discrete speed step
  int t_discrete;  // discrete time step
  // TODO(weijun): add heading.
  MotionSample(double v, double t)
      : v_discrete(std::max(0, RoundToInt(v / kSpeedSampleStep))),
        t_discrete(std::clamp(RoundToInt(t / kDiscreteTimeSampleStep), 0,
                              kDiscreteTimeHorizon)) {}

  friend bool operator==(const MotionSample& lhs, const MotionSample& rhs) {
    return lhs.v_discrete == rhs.v_discrete && lhs.t_discrete == rhs.t_discrete;
  }

  template <typename H>
  friend H AbslHashValue(H h, const MotionSample& ms) {
    return H::combine(std::move(h), ms.v_discrete, ms.t_discrete);
  }
};

class MotionGraph {
 public:
  explicit MotionGraph(const GeometryGraph* geom_graph)
      : geometry_graph_(geom_graph) {}
  virtual int node_size() const = 0;
  virtual int edge_size() const = 0;
  virtual const MotionNode& GetMotionNode(MotionNodeIndex i) const = 0;
  virtual const MotionEdge& GetMotionEdge(MotionEdgeIndex i) const = 0;
  virtual MotionEdge* GetMutableMotionEdge(MotionEdgeIndex i) = 0;
  virtual absl::Span<const MotionEdgeIndex> GetOutgoingEdges(
      MotionNodeIndex i) const = 0;
  virtual MotionNodeIndex AddMotionNode(MotionState node,
                                        GeometryNodeIndex geom_index) = 0;
  virtual MotionEdgeIndex AddMotionEdge(MotionNodeIndex start_node_index,
                                        MotionNodeIndex end_node_index,
                                        const MotionForm* motion_form,
                                        GeometryNodeIndex end_geom_index,
                                        MotionEdgeIndex prev_edge) = 0;

  virtual const GeometryGraph* geometry_graph() const {
    return geometry_graph_;
  }

  virtual ~MotionGraph() {}

  virtual void ToProto(MotionGraphProto* proto) const = 0;

 protected:
  const GeometryGraph* geometry_graph_;
};

class XYTMotionGraph : public MotionGraph {
 public:
  explicit XYTMotionGraph(const GeometryGraph* geom_graph)
      : MotionGraph(geom_graph) {}

  int node_size() const override { return nodes_.size(); }
  int edge_size() const override { return edges_.size(); }

  const MotionNode& GetMotionNode(MotionNodeIndex i) const override {
    return nodes_[i];
  }
  const MotionEdge& GetMotionEdge(MotionEdgeIndex i) const override {
    return edges_[i];
  }
  MotionEdge* GetMutableMotionEdge(MotionEdgeIndex i) override {
    return &edges_[i];
  }

  absl::Span<const MotionEdgeIndex> GetOutgoingEdges(
      MotionNodeIndex i) const override {
    QCHECK(outgoing_edges_.valid_index(i));
    return outgoing_edges_[i];
  }
  MotionNodeIndex AddMotionNode(MotionState node,
                                GeometryNodeIndex geom_index) override;
  MotionEdgeIndex AddMotionEdge(MotionNodeIndex start_node_index,
                                MotionNodeIndex end_node_index,
                                const MotionForm* motion_form,
                                GeometryNodeIndex end_geom_index,
                                MotionEdgeIndex prev_edge) override;

  void ToProto(MotionGraphProto* proto) const override;

 private:
  std::vector<MotionEdgeIndex>* GetOrCreateOutgoingEdge(MotionNodeIndex i);

  MotionNodeVector<MotionNode> nodes_;
  MotionNodeVector<std::vector<MotionEdgeIndex>> outgoing_edges_;
  MotionEdgeVector<MotionEdge> edges_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_H_
