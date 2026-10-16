// Bridge from marker TF frames to rtabmap_msgs/LandmarkDetections.
//
// For each configured marker id we look up base_link <- marker_frame in TF,
// accumulate a sliding window of samples, and publish the running median pose
// as a landmark detection. Outlier samples that jump beyond `outlier_gate_m`
// are dropped unless they persist for `outlier_persist_count` ticks in a row
// (which means the marker was likely re-observed after occlusion). Stale TF
// (older than `max_tf_age_sec`) is treated the same as "marker not visible".

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rtabmap_msgs/msg/landmark_detection.hpp>
#include <rtabmap_msgs/msg/landmark_detections.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace tf_to_fiducial
{

enum class OperationMode { Mapping, Localization };

struct PoseSample
{
  double px{0}, py{0}, pz{0};
  double ox{0}, oy{0}, oz{0}, ow{0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct MarkerPose
{
  std::string marker_frame_id;
  double px{0}, py{0}, pz{0};
  double ox{0}, oy{0}, oz{0}, ow{0};
};

namespace
{

double median_scalar(std::vector<double> values)
{
  const std::size_t n = values.size();
  if (n == 0) {
    return 0.0;
  }
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(n / 2);
  std::nth_element(values.begin(), mid, values.end());
  if (n % 2 == 1) {
    return *mid;
  }
  const double lo = *std::max_element(values.begin(), mid);
  return (lo + *mid) * 0.5;
}

template <class Extract>
double median_of(const std::vector<PoseSample> & samples, Extract get)
{
  std::vector<double> v;
  v.reserve(samples.size());
  for (const auto & s : samples) {
    v.push_back(get(s));
  }
  return median_scalar(std::move(v));
}

MarkerPose median_pose_from_samples(
  const std::vector<PoseSample> & samples, const std::string & marker_frame_id)
{
  MarkerPose out;
  out.marker_frame_id = marker_frame_id;
  if (samples.empty()) {
    return out;
  }

  out.px = median_of(samples, [](const PoseSample & s) { return s.px; });
  out.py = median_of(samples, [](const PoseSample & s) { return s.py; });
  out.pz = median_of(samples, [](const PoseSample & s) { return s.pz; });

  // Align every quaternion to the sign of the first before taking a
  // component-wise median (double-cover / hemisphere fix).
  std::vector<std::array<double, 4>> q(samples.size());
  q[0] = {samples[0].ox, samples[0].oy, samples[0].oz, samples[0].ow};
  for (std::size_t i = 1; i < samples.size(); ++i) {
    std::array<double, 4> qi = {samples[i].ox, samples[i].oy, samples[i].oz, samples[i].ow};
    const double dot = q[0][0] * qi[0] + q[0][1] * qi[1] + q[0][2] * qi[2] + q[0][3] * qi[3];
    if (dot < 0.0) {
      for (double & c : qi) {
        c = -c;
      }
    }
    q[i] = qi;
  }

  auto median_component = [&](std::size_t idx) {
    std::vector<double> v;
    v.reserve(q.size());
    for (const auto & qi : q) {
      v.push_back(qi[idx]);
    }
    return median_scalar(std::move(v));
  };
  out.ox = median_component(0);
  out.oy = median_component(1);
  out.oz = median_component(2);
  out.ow = median_component(3);

  const double norm = std::sqrt(out.ox * out.ox + out.oy * out.oy + out.oz * out.oz + out.ow * out.ow);
  if (norm > 1e-12) {
    out.ox /= norm;
    out.oy /= norm;
    out.oz /= norm;
    out.ow /= norm;
  } else {
    out.ox = out.oy = out.oz = 0.0;
    out.ow = 1.0;
  }
  return out;
}

PoseSample sample_from_transform(const geometry_msgs::msg::TransformStamped & tf_st)
{
  PoseSample s;
  s.px = tf_st.transform.translation.x;
  s.py = tf_st.transform.translation.y;
  s.pz = tf_st.transform.translation.z;
  s.ox = tf_st.transform.rotation.x;
  s.oy = tf_st.transform.rotation.y;
  s.oz = tf_st.transform.rotation.z;
  s.ow = tf_st.transform.rotation.w;
  s.stamp = rclcpp::Time(tf_st.header.stamp);
  return s;
}

void fill_landmark_detection(
  rtabmap_msgs::msg::LandmarkDetection & ld,
  const rclcpp::Time & stamp,
  const std::string & reference_frame,
  const MarkerPose & entry,
  int id,
  double marker_size)
{
  ld.header.stamp = stamp;
  ld.header.frame_id = reference_frame;
  ld.landmark_frame_id = entry.marker_frame_id;
  ld.id = id;
  ld.size = marker_size;
  ld.pose.pose.position.x = entry.px;
  ld.pose.pose.position.y = entry.py;
  ld.pose.pose.position.z = entry.pz;
  ld.pose.pose.orientation.x = entry.ox;
  ld.pose.pose.orientation.y = entry.oy;
  ld.pose.pose.orientation.z = entry.oz;
  ld.pose.pose.orientation.w = entry.ow;
}

OperationMode parse_operation_mode_param(const std::string & value)
{
  std::string v = value;
  std::transform(v.begin(), v.end(), v.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return (v == "localization" || v == "loc") ? OperationMode::Localization : OperationMode::Mapping;
}

}  // namespace

class TfToFiducialNode : public rclcpp::Node
{
public:
  TfToFiducialNode()
  : rclcpp::Node("tf_to_fiducial_node")
  {
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    marker_frame_prefix_ = declare_parameter<std::string>("marker_frame_prefix", "stag_hd19_");
    update_rate_ = declare_parameter<double>("update_rate", 10.0);
    output_topic_ = declare_parameter<std::string>("output_topic", "/landmark_detections");
    marker_size_ = declare_parameter<double>("marker_size", 0.1);
    marker_id_start_ = declare_parameter<int>("marker_id_range_start", 1);
    marker_id_end_ = declare_parameter<int>("marker_id_range_end", 3000);
    mapping_pose_sample_count_ = declare_parameter<int>("mapping_pose_sample_count", 5);
    localization_pose_sample_count_ = declare_parameter<int>("localization_pose_sample_count", 2);
    max_publish_distance_m_ = declare_parameter<double>("max_publish_distance_m", 9.0);
    // lookupTransform(..., TimePointZero) returns the newest TF regardless of
    // age, so a stale (still-cached) transform can be published as if the
    // marker were currently visible. Drop TFs older than max_tf_age_sec.
    max_tf_age_sec_ = declare_parameter<double>("max_tf_age_sec", 0.3);
    // Sample-level jump gate: reject samples that jump beyond outlier_gate_m
    // from the current window median, unless outlier_persist_count similar
    // "outliers" arrive in a row (re-observation after occlusion).
    outlier_gate_m_ = declare_parameter<double>("outlier_gate_m", 0.25);
    outlier_persist_count_ = declare_parameter<int>("outlier_persist_count", 2);
    operation_mode_param_ = declare_parameter<std::string>("operation_mode", "mapping");

    // Non-empty allowlist restricts published markers to these IDs (ignores id_range).
    const auto allowed_ids_vec =
      declare_parameter<std::vector<int>>("allowed_marker_ids", std::vector<int>{});
    allowed_ids_.insert(allowed_ids_vec.begin(), allowed_ids_vec.end());

    marker_id_start_ = std::max(1, marker_id_start_);
    mapping_pose_sample_count_ = std::max(1, mapping_pose_sample_count_);
    localization_pose_sample_count_ = std::max(1, localization_pose_sample_count_);
    marker_id_end_ = std::max(marker_id_start_, marker_id_end_);
    operation_mode_ = parse_operation_mode_param(operation_mode_param_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    pub_ = create_publisher<rtabmap_msgs::msg::LandmarkDetections>(output_topic_, 10);

    if (update_rate_ <= 0.0) {
      update_rate_ = 10.0;
    }
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / update_rate_),
      std::bind(&TfToFiducialNode::on_timer, this));

    diagnostic_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(5.0),
      std::bind(&TfToFiducialNode::diagnostic_callback, this));

    RCLCPP_INFO(get_logger(),
      "tf_to_fiducial_node started: mode=%s base_frame=%s prefix=%s topic=%s rate=%.2fHz id_range=%d..%d",
      operation_mode_ == OperationMode::Localization ? "localization" : "mapping",
      robot_base_frame_.c_str(), marker_frame_prefix_.c_str(), output_topic_.c_str(),
      update_rate_, marker_id_start_, marker_id_end_);
  }

  void init_tf_listener()
  {
    if (!tf_listener_) {
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this());
      RCLCPP_INFO(get_logger(), "TransformListener created");
    }
  }

private:
  void diagnostic_callback()
  {
    // Are any marker frames visible at all? Cheap heuristic: scan TF frames.
    bool any_marker = false;
    for (const auto & frame : tf_buffer_->getAllFrameNames()) {
      if (frame.rfind(marker_frame_prefix_, 0) == 0) {
        any_marker = true;
        break;
      }
    }
    if (!any_marker) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "No %s* markers visible yet (robot_base_frame=%s). Normal if robot hasn't reached markers.",
        marker_frame_prefix_.c_str(), robot_base_frame_.c_str());
    }
  }

  bool within_publish_distance(const MarkerPose & marker_pose) const
  {
    if (max_publish_distance_m_ <= 0.0) {
      return true;
    }
    const double dist2 =
      marker_pose.px * marker_pose.px + marker_pose.py * marker_pose.py + marker_pose.pz * marker_pose.pz;
    return dist2 <= max_publish_distance_m_ * max_publish_distance_m_;
  }

  // Candidate marker IDs for this tick: the explicit allowlist if set,
  // otherwise every frame currently present in TF whose name matches
  // "<prefix><integer>". Avoids blindly probing the whole ID range.
  std::vector<int> candidate_ids()
  {
    if (!allowed_ids_.empty()) {
      return {allowed_ids_.begin(), allowed_ids_.end()};
    }
    std::vector<int> ids;
    for (const auto & frame : tf_buffer_->getAllFrameNames()) {
      if (frame.rfind(marker_frame_prefix_, 0) != 0) {
        continue;
      }
      const std::string suffix = frame.substr(marker_frame_prefix_.size());
      if (suffix.empty() ||
        !std::all_of(suffix.begin(), suffix.end(),
                     [](unsigned char c) { return std::isdigit(c); }))
      {
        continue;
      }
      ids.push_back(std::stoi(suffix));
    }
    return ids;
  }

  void on_timer()
  {
    const int window_size = (operation_mode_ == OperationMode::Localization)
      ? localization_pose_sample_count_
      : mapping_pose_sample_count_;
    const bool use_distance_filter = (operation_mode_ == OperationMode::Localization);

    std::vector<rtabmap_msgs::msg::LandmarkDetection> dets;
    std::unordered_set<int> visible_ids;

    for (int id : candidate_ids()) {
      if (id < marker_id_start_ || id > marker_id_end_) {
        continue;
      }
      const std::string marker_frame = marker_frame_prefix_ + std::to_string(id);
      geometry_msgs::msg::TransformStamped tf_st;
      try {
        tf_st = tf_buffer_->lookupTransform(robot_base_frame_, marker_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        continue;
      }
      if ((get_clock()->now() - rclcpp::Time(tf_st.header.stamp)).seconds() > max_tf_age_sec_) {
        continue;
      }
      visible_ids.insert(id);
      accumulate_sample_and_maybe_publish(
        id, marker_frame, sample_from_transform(tf_st),
        window_size, use_distance_filter, dets);
    }

    forget_missing(localization_pending_, visible_ids);
    forget_missing(outlier_streak_, visible_ids);

    if (!dets.empty()) {
      rtabmap_msgs::msg::LandmarkDetections msg;
      msg.landmarks = std::move(dets);
      pub_->publish(msg);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Publishing landmark(s), last batch: %zu", msg.landmarks.size());
    }
  }

  void accumulate_sample_and_maybe_publish(
    int id,
    const std::string & marker_frame,
    const PoseSample & sample,
    int window_size,
    bool use_distance_filter,
    std::vector<rtabmap_msgs::msg::LandmarkDetection> & dets)
  {
    auto & window = localization_pending_[id];
    auto & streak = outlier_streak_[id];

    if (window.empty()) {
      window.push_back(sample);
    } else {
      const std::vector<PoseSample> cur(window.begin(), window.end());
      const MarkerPose cur_med = median_pose_from_samples(cur, marker_frame);
      const double dx = sample.px - cur_med.px;
      const double dy = sample.py - cur_med.py;
      const double dz = sample.pz - cur_med.pz;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) > outlier_gate_m_) {
        streak.push_back(sample);
        if (static_cast<int>(streak.size()) < outlier_persist_count_) {
          // Isolated outlier — drop it, keep window untouched.
          return;
        }
        // A confirmed run of outliers: treat as a real new position (marker
        // re-seen from a different angle) and restart the window with them.
        window.clear();
        for (const auto & s : streak) {
          window.push_back(s);
        }
        streak.clear();
      } else {
        streak.clear();
        window.push_back(sample);
      }
    }

    while (static_cast<int>(window.size()) > window_size) {
      window.pop_front();
    }
    if (static_cast<int>(window.size()) != window_size) {
      return;
    }

    const std::vector<PoseSample> samples(window.begin(), window.end());
    const MarkerPose pose = median_pose_from_samples(samples, marker_frame);
    if (use_distance_filter && !within_publish_distance(pose)) {
      return;
    }
    rtabmap_msgs::msg::LandmarkDetection ld;
    fill_landmark_detection(ld, samples.back().stamp, robot_base_frame_, pose, id, marker_size_);
    dets.push_back(std::move(ld));
  }

  template <class Map>
  static void forget_missing(Map & m, const std::unordered_set<int> & visible)
  {
    for (auto it = m.begin(); it != m.end(); ) {
      it = visible.count(it->first) ? std::next(it) : m.erase(it);
    }
  }

  OperationMode operation_mode_{OperationMode::Mapping};
  std::string operation_mode_param_;
  std::string robot_base_frame_;
  std::string marker_frame_prefix_;
  double update_rate_{10.0};
  std::string output_topic_;
  double marker_size_{0.1};
  int marker_id_start_{1};
  int marker_id_end_{3000};
  std::set<int> allowed_ids_{};
  int mapping_pose_sample_count_{5};
  int localization_pose_sample_count_{2};
  double max_publish_distance_m_{9.0};
  double max_tf_age_sec_{0.3};
  double outlier_gate_m_{0.25};
  int outlier_persist_count_{2};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<rtabmap_msgs::msg::LandmarkDetections>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  std::map<int, std::deque<PoseSample>> localization_pending_;
  std::map<int, std::deque<PoseSample>> outlier_streak_;
};

}  // namespace tf_to_fiducial

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tf_to_fiducial::TfToFiducialNode>();
  node->init_tf_listener();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
