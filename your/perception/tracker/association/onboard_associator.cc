#include "onboard/perception/tracker/association/onboard_associator.h"

#include "google/protobuf/text_format.h"

namespace qcraft::tracker {
namespace association {

template <typename T, typename M>
bool OnboardAssociator<T, M>::Init(const std::string config_file,
                                   const std::string msg) {
  association::AssociationProto config;
  QCHECK(file_util::TextFileToProto(config_file, &config));
  std::string init_log;
  google::protobuf::TextFormat::PrintToString(config, &init_log);
  QLOG(INFO) << absl::StrFormat(
      "*** Init %s Associator ***\n"
      "%s"
      "*****************************\n",
      msg.c_str(), init_log.c_str());
  config_ = config;
  QCHECK(config_.has_similarity_matrix_builder());
  QCHECK(config_.has_match_solver());
  return similarity_matrix_builder_.Init(config_.similarity_matrix_builder()) &&
         match_solver_.Init(config_.match_solver());
}

template class OnboardAssociator<Track<TrackState>, MeasurementProto>;
template class OnboardAssociator<
    Track<multi_camera_fusion::MultiCameraTrackState>, MeasurementProto>;

}  // namespace association
}  // namespace qcraft::tracker
