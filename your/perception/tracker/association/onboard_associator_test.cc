#include "onboard/perception/tracker/association/onboard_associator.h"

#include "glog/logging.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

DEFINE_bool(generate_config, false,
            "Use \"TEST(GenerateConfig)\" to generate config string");

namespace qcraft::tracker {
namespace association {

TEST(OnboardAssociator, GenerateConfig) {
  // Configure Association
  association::AssociationProto association_proto;

  auto similarity_matrix_builder_proto =
      association_proto.mutable_similarity_matrix_builder();
  // Configure Gates
  auto gates = similarity_matrix_builder_proto->mutable_gates();
  gates->add_gate_params();
  gates->mutable_gate_params(0)->set_gate_type(GatesProto::TYPE);
  gates->mutable_gate_params(0)->add_gate_thresholds(0.5);

  gates->add_gate_params();
  gates->mutable_gate_params(1)->set_gate_type(GatesProto::TYPE_CIRCLE);
  gates->mutable_gate_params(1)->add_gate_thresholds(Sqr(1.0));
  gates->mutable_gate_params(1)->add_gate_thresholds(Sqr(7.0));

  gates->add_gate_params();
  gates->mutable_gate_params(2)->set_gate_type(GatesProto::ELIPSE);
  gates->mutable_gate_params(2)->add_gate_thresholds(4.99);

  gates->add_gate_params();
  gates->mutable_gate_params(3)->set_gate_type(GatesProto::BEV_IOU);
  gates->mutable_gate_params(3)->add_gate_thresholds(1e-8);

  gates->add_gate_params();
  gates->mutable_gate_params(4)->set_gate_type(GatesProto::SPEED);
  gates->mutable_gate_params(4)->add_gate_thresholds(0.0);

  // Configure metrics
  auto metrics = similarity_matrix_builder_proto->mutable_metrics();
  metrics->add_metric_params();
  metrics->mutable_metric_params(0)->set_metric_type(
      MetricsProto::EUCLIDEAN_DIST);
  metrics->mutable_metric_params(0)->set_metric_weight(0.5);
  metrics->mutable_metric_params(0)->add_metric_thresholds(0.0);

  metrics->add_metric_params();
  metrics->mutable_metric_params(1)->set_metric_type(MetricsProto::VEHICLE_IOU);
  metrics->mutable_metric_params(1)->set_metric_weight(0.5);
  metrics->mutable_metric_params(1)->add_metric_thresholds(0.0);

  // Configure MatchSolverProto
  auto match_solver = association_proto.mutable_match_solver();
  match_solver->set_solver_algorithm(association::MatchSolverProto::GNN);

  // Print config string to console.
  std::string config;
  google::protobuf::TextFormat::PrintToString(association_proto, &config);
  printf(
      "***** Copy Config Below*****\n"
      "%s"
      "****************************\n",
      config.c_str());
  if (FLAGS_generate_config) {
    FILE* fp = fopen(
        "onboard/perception/tracker/association/config/"
        "associator_assemble_config.pb.txt",
        "w");
    if (fp) {
      fprintf(fp, "%s", config.c_str());
      fclose(fp);
    } else {
      QLOG(FATAL) << "Open file "
                     "onboard/perception/tracker/association/config/"
                     "associator_assemble_config.pb.txt failed!";
    }
  }
}

TEST(OnboardAssociator, TestAssociation) {
  ThreadPool thread_pool(2);
  OnboardAssociator<Track<TrackState>, MeasurementProto> onboard_associator(
      &thread_pool);
  onboard_associator.Init(
      "onboard/perception/tracker/association/config/"
      "dbq_laser_associator_assemble_config.pb.txt",
      "DBQ_LIDAR");
  onboard_associator.Init(
      "onboard/perception/tracker/association/config/"
      "dbq_radar_associator_assemble_config.pb.txt",
      "DBQ_RADAR");
  onboard_associator.Init(
      "onboard/perception/tracker/association/config/"
      "dbq_camera_associator_assemble_config.pb.txt",
      "DBQ_CAMERA");
}

}  // namespace association
}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
