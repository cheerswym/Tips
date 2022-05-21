#include "onboard/perception/human_pipeline_tracker.h"

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "gtest/gtest.h"

DEFINE_bool(pcn_track_visualize, false, "Visualize and save the track image");

namespace qcraft {

namespace {
std::map<int, int> idx_to_now = {{0, 2}, {1, 0}, {2, 1}};
}

bool GetOneInput(
    FILE *ifile, double *detection_timestamp,
    std::vector<FieryEyeNetClassifier::DetectionBox> *boxes,
    std::vector<ImagePatchClassifier::ClassificationResult> *pcn_box_types,
    std::vector<int> *box_labels) {
  if (feof(ifile)) return false;
  int boxes_num;
  (void)fscanf(ifile, "%d %lf\n", &boxes_num, detection_timestamp);
  for (int i = 0; i < boxes_num; i++) {
    double box_center_x, box_center_y, box_heading, box_length, box_width,
        velocity_x, velocity_y;
    int pcn_type, box_type, label;
    (void)fscanf(ifile, "%lf %lf %lf %lf %lf %lf %lf %d %d %d \n",
                 &box_center_x, &box_center_y, &box_heading, &box_length,
                 &box_width, &velocity_x, &velocity_y, &pcn_type, &box_type,
                 &label);
    FieryEyeNetClassifier::DetectionBox box;
    box.box =
        Box2d({box_center_x, box_center_y}, box_heading, box_length, box_width);
    box.velocity = {velocity_x, velocity_y};
    boxes->push_back(box);
    pcn_box_types->push_back(
        static_cast<ImagePatchClassifier::ClassificationResult>(
            idx_to_now[pcn_type]));
    box_labels->push_back(idx_to_now[label]);
  }
  return true;
}

int MapType(ImagePatchClassifier::ClassificationResult type) {
  // Because currently cyclist is also considered to be a kind of pedestrian,
  // but the output of pcn is still differentiated. In the current test,
  // it is believed that as long as there is someone in this box,
  // it will be output as pedestrian

  return type == static_cast<int>(
                     ImagePatchClassifier::ClassificationResult::kCyclist)
             ? static_cast<int>(
                   ImagePatchClassifier::ClassificationResult::kPedestrian)
             : static_cast<int>(type);
}

std::pair<Vec2d, Vec2d> GetXYRange(const char boxes_info_path[]) {
  FILE *ifile = fopen(boxes_info_path, "r");
  const double inf = std::numeric_limits<double>::infinity();
  double x_min = inf, x_max = -inf, y_min = inf, y_max = -inf;
  while (!feof(ifile)) {
    int boxes_num = 0;
    double detection_timestamp = 0;
    (void)fscanf(ifile, "%d %lf\n", &boxes_num, &detection_timestamp);
    for (int i = 0; i < boxes_num; i++) {
      double box_center_x, box_center_y, box_heading, box_length, box_width,
          velocity_x, velocity_y;
      int pcn_type = 0, box_type = 0, label = 0;
      (void)fscanf(ifile, "%lf %lf %lf %lf %lf %lf %lf %d %d %d \n",
                   &box_center_x, &box_center_y, &box_heading, &box_length,
                   &box_width, &velocity_x, &velocity_y, &pcn_type, &box_type,
                   &label);
      x_min = std::min(x_min, box_center_x);
      x_max = std::max(x_max, box_center_x);
      y_min = std::min(y_min, box_center_y);
      y_max = std::max(y_max, box_center_y);
    }
  }
  std::pair<Vec2d, Vec2d> xy_range;
  xy_range.first = Vec2d(x_min, y_min);
  xy_range.second = Vec2d(x_max, y_max);
  fclose(ifile);
  return xy_range;
}

void Visualize(const std::pair<Vec2d, Vec2d> &xy_range,
               const std::vector<FieryEyeNetClassifier::DetectionBox> &boxes,
               const std::vector<ImagePatchClassifier::ClassificationResult>
                   &pcn_box_types,
               const std::vector<ImagePatchClassifier::ClassificationResult>
                   &track_box_types,
               const std::vector<int> &box_labels,
               const std::vector<std::string> &track_infos, int frame) {
  // define the backgroud mat
  constexpr int kBackgroudSize = 1000;
  constexpr int kBoxLenth = 50;
  cv::Size bg_size(kBackgroudSize, kBackgroudSize);
  // make sure all box and info could be showed competely
  Vec2d img_size(kBackgroudSize - 4 * kBoxLenth,
                 kBackgroudSize - 4 * kBoxLenth);
  cv::Mat background(bg_size, CV_8UC3, cv::Scalar(255, 255, 255));
  // change to canvas coordinate
  // because info text on the top of the box, align to the left,
  // in order to display the text completely, add some offset to box_y
  for (int i = 0; i < boxes.size(); i++) {
    int box_x = static_cast<int>(boxes[i].box.center_x() - xy_range.first.x()) *
                img_size.x() / (xy_range.second.x() - xy_range.first.x());
    int box_y = static_cast<int>(boxes[i].box.center_y() - xy_range.first.y()) *
                    img_size.y() / (xy_range.second.y() - xy_range.first.y()) +
                kBoxLenth;

    cv::Rect rect(box_x, box_y, kBoxLenth, kBoxLenth);
    cv::Scalar colors[3] = {cv::Scalar(0, 0, 0), cv::Scalar(0, 255, 0),
                            cv::Scalar(255, 0, 0)};
    cv::Scalar box_color = track_box_types[i] == box_labels[i]
                               ? colors[box_labels[i]]
                               : cv::Scalar(0, 0, 255);
    cv::rectangle(background, rect, box_color, 1, cv::LINE_8, 0);

    int track_id = 0;
    char box_info[50];
    sscanf(track_infos[i].c_str(), "track:id%d,", &track_id);
    snprintf(box_info, sizeof(box_info), "id:%d pcn:%d track:%d label:%d",
             track_id, pcn_box_types[i], track_box_types[i], box_labels[i]);
    cv::putText(background, box_info, cv::Point(box_x, box_y),
                cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 0, 1), 1,
                cv::LINE_8);
  }
  char img_name[100];
  snprintf(img_name, sizeof(img_name),
           "onboard/perception/testdata/human_pipeline_tracker/%d.jpg", frame);
  cv::imwrite(img_name, background);
}

TEST(PcnTrackTrtTest, UnitTestTrt) {
  constexpr char boxes_info_path[] =
      "onboard/perception/testdata/human_pipeline_tracker/track_boxes.txt";
  FILE *ifile = fopen(boxes_info_path, "r");
  double detection_timestamp = 0;
  human_tracking::HumanTracker tracker;
  std::vector<std::string> track_infos(50);
  std::vector<FieryEyeNetClassifier::DetectionBox> boxes;
  std::vector<ImagePatchClassifier::ClassificationResult> pcn_box_types;
  std::vector<int> box_labels;
  int frame = 0;

  std::pair<Vec2d, Vec2d> xy_range = GetXYRange(boxes_info_path);

  while (GetOneInput(ifile, &detection_timestamp, &boxes, &pcn_box_types,
                     &box_labels)) {
    std::vector<ImagePatchClassifier::ClassificationResult> track_box_types;
    track_box_types = tracker.GetHumanTrackingResult(
        boxes, detection_timestamp, pcn_box_types, &track_infos);
    if (FLAGS_pcn_track_visualize) {
      Visualize(xy_range, boxes, pcn_box_types, track_box_types, box_labels,
                track_infos, frame++);
    }

    for (int i = 0; i < track_box_types.size(); i++) {
      EXPECT_EQ(MapType(track_box_types[i]), box_labels[i]);
    }
    boxes.clear();
    pcn_box_types.clear();
    box_labels.clear();
  }
  fclose(ifile);
}
}  // namespace qcraft
