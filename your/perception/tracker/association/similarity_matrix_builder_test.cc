#include "onboard/perception/tracker/association/similarity_matrix_builder.h"

#include "glog/logging.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {
namespace association {

TEST(SimilarityMatrixBuilder, TestBuildSimilarityMatrix) {
  std::vector<TrackRef> tracks;
  std::vector<const MeasurementProto*> measurments;
  Eigen::MatrixXd similarity_matrix;
  ThreadPool therad_pool(2);
  SemanticMapManager semantic_map_manager;
  CoordinateConverter coordinate_converter;
  SimilarityMatrixBuilder<Track<TrackState>, MeasurementProto> builder(
      &therad_pool);

  association::AssociationProto config;
  EXPECT_TRUE(file_util::TextFileToProto(
      "onboard/perception/tracker/association/config/"
      "dbq_laser_associator_assemble_config.pb.txt",
      &config));
  EXPECT_TRUE(config.has_similarity_matrix_builder());
  association::SimilarityMatrixBuilderProto config_proto =
      config.similarity_matrix_builder();

  EXPECT_TRUE(builder.Init(config_proto));
  TrackerDebugProto_AssociationDebugInfoProto debug_proto;
  builder.BuildSimilarityMatrix(0.0, measurments, tracks, &coordinate_converter,
                                &semantic_map_manager, &similarity_matrix,
                                &debug_proto);
  builder.BuildSimilarityMatrix(0.0, measurments, tracks, nullptr, nullptr,
                                &similarity_matrix, nullptr);
}

}  // namespace association
}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
