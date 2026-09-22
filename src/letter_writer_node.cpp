// letter_writer_node.cpp
//
// Luong xu ly:
//   1) Doc tham so (chu can ve, kich thuoc, vi tri mat phang, ...)
//   2) Lay danh sach net (strokes) cua chu cai tu ur3_letter_writer::getLetterStrokes
//   3) Chuyen tung diem (u, v) chuan hoa -> Pose Cartesian thuc te (x, y, z, orientation)
//   4) Voi moi net:
//        - Di chuyen (khong cham mat phang) toi phia tren diem dau (pen up)
//        - Ha dau cong tac xuong mat phang (pen down) bang Cartesian path ngan
//        - Ve net bang MoveGroupInterface::computeCartesianPath() qua cac diem con lai
//        - Nhac dau cong tac len (pen up) truoc khi sang net tiep theo
//   5) Publish marker de xem truoc hinh dang chu tren RViz (topic /letter_preview)
//   6) Trong luc robot chuyen dong, doc vi tri THUC TE cua end-effector qua TF
//      va noi dai dan mot LINE_STRIP marker (topic /letter_trace) -> RViz ve lai
//      dung "vet but" ma dau cong tac da di qua, thay vi chi xem truoc tinh

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "ur3_letter_writer/letters.hpp"

using namespace std::chrono_literals;

namespace
{

// Cau hinh mat phang ve chu, dung chung cho toan bo node
struct PlaneConfig
{
  double plane_x;    // khoang cach (m) tu goc robot toi mat phang, theo truc X
  double pen_lift;   // do nang dau cong tac khi "pen up" (m)
  double center_y;   // tam chu theo truc Y (m)
  double center_z;   // tam chu theo truc Z (m)
  double width;      // be rong chu (m)
  double height;      // chieu cao chu (m)
  geometry_msgs::msg::Quaternion orientation;  // huong dau cong tac (co dinh)
};

// Chuyen 1 diem chuan hoa (u, v) trong [0,1]x[0,1] sang Pose Cartesian thuc te.
// pen_down = true  -> dau cong tac cham mat phang (x = plane_x)
// pen_down = false -> dau cong tac nhac len (x = plane_x - pen_lift)
geometry_msgs::msg::Pose makePose(double u, double v, bool pen_down, const PlaneConfig & cfg)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pen_down ? cfg.plane_x : cfg.plane_x - cfg.pen_lift;
  pose.position.y = cfg.center_y - cfg.width / 2.0 + u * cfg.width;
  pose.position.z = cfg.center_z - cfg.height / 2.0 + v * cfg.height;
  pose.orientation = cfg.orientation;
  return pose;
}

// Lap ke hoach + thuc thi 1 doan Cartesian ngan qua danh sach waypoints.
// Tra ve false neu ty le hoan thanh (fraction) qua thap -> bo qua de an toan.
bool executeCartesianSegment(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  double eef_step,
  const rclcpp::Logger & logger)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  // jump_threshold = 0.0 -> tat kiem tra buoc nhay khop (khuyen nghi tu MoveIt2,
  // kiem tra nay khong dang tin cay va da bi loai bo o cac ban moi hon)
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, 0.0, trajectory);
  RCLCPP_INFO(logger, "    computeCartesianPath fraction = %.2f", fraction);

  if (fraction < 0.95) {
    RCLCPP_WARN(logger, "    Fraction qua thap (<0.95) -> bo qua doan nay de tranh IK/va cham.");
    return false;
  }

  auto result = move_group.execute(trajectory);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(logger, "    Thuc thi trajectory that bai.");
    return false;
  }
  return true;
}


// Ghi lai "vet but" THUC TE cua end-effector.
//
// Chay tren mot thread rieng song song voi move_group.move()/execute()
// (deu la lenh blocking): moi chu ky 1/rate_hz giay, doc TF
// planning_frame -> ee_frame de lay vi tri that cua dau cong tac, roi noi
// dai dan mot marker LINE_STRIP va publish lai. Nho vay RViz ve ra quy dao
// dung bang duong ma robot thuc su di, ngay trong luc no dang chuyen dong.
//
// Moi net chu duoc ghi vao mot marker id RIENG (beginSegment(id)) de cac net
// truoc van con nguyen tren man hinh va khong bi noi lien voi net sau bang
// mot doan thang "ma" khi dau cong tac duoc nhac len di sang net moi.
class EndEffectorTracer
{
public:
  EndEffectorTracer(
    rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    std::string planning_frame,
    std::string ee_frame,
    double rate_hz,
    double line_width)
  : node_(std::move(node)),
    pub_(std::move(pub)),
    tf_buffer_(node_->get_clock()),
    // spin_thread=false: dung chung executor da spin `node`, tranh loi
    // "node da duoc them vao mot executor khac" khi TransformListener tu
    // tao executor rieng cho cung mot node.
    tf_listener_(tf_buffer_, node_, false),
    planning_frame_(std::move(planning_frame)),
    ee_frame_(std::move(ee_frame)),
    rate_hz_(rate_hz)
  {
    marker_.header.frame_id = planning_frame_;
    marker_.ns = "letter_trace";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.scale.x = line_width;
    marker_.color.r = 0.05;
    marker_.color.g = 0.9;
    marker_.color.b = 1.0;
    marker_.color.a = 1.0;
    marker_.pose.orientation.w = 1.0;
  }

  ~EndEffectorTracer() { stop(); }

  EndEffectorTracer(const EndEffectorTracer &) = delete;
  EndEffectorTracer & operator=(const EndEffectorTracer &) = delete;

  void start()
  {
    if (running_.exchange(true)) {
      return;
    }
    thread_ = std::thread(&EndEffectorTracer::loop, this);
  }

  // Mo mot doan vet moi (marker id rieng) va bat dau ghi
  void beginSegment(int id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    marker_.id = id;
    marker_.points.clear();
    recording_.store(true);
  }

  // Tam dung ghi (vd khi nhac dau cong tac di sang net khac)
  void pauseSegment() { recording_.store(false); }

  void stop()
  {
    recording_.store(false);
    if (!running_.exchange(false)) {
      return;
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void loop()
  {
    rclcpp::Rate rate(rate_hz_);
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    while (running_.load() && rclcpp::ok()) {
      if (recording_.load()) {
        try {
          const auto tf = tf_buffer_.lookupTransform(planning_frame_, ee_frame_, tf2::TimePointZero);
          geometry_msgs::msg::Point p;
          p.x = tf.transform.translation.x;
          p.y = tf.transform.translation.y;
          p.z = tf.transform.translation.z;

          std::lock_guard<std::mutex> lock(mutex_);
          // Chi them diem khi dau cong tac da di duoc it nhat kMinStep:
          // luc robot dung yen (cho lap ke hoach) se khong lam marker phinh
          // to vo ich, nhung van du day de duong cong muot.
          if (recording_.load() && (marker_.points.empty() || moved(marker_.points.back(), p))) {
            marker_.points.push_back(p);
            marker_.header.stamp = node_->now();
            pub_->publish(marker_);
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), steady_clock, 2000,
            "Khong lay duoc TF %s -> %s de ve vet quy dao: %s",
            planning_frame_.c_str(), ee_frame_.c_str(), ex.what());
        }
      }
      rate.sleep();
    }
  }

  static bool moved(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    constexpr double kMinStep = 0.0005;  // 0.5 mm
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz > kMinStep * kMinStep;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string planning_frame_;
  std::string ee_frame_;
  double rate_hz_;

  visualization_msgs::msg::Marker marker_;
  std::mutex mutex_;
  std::atomic<bool> running_{false};
  std::atomic<bool> recording_{false};
  std::thread thread_;
};

// Publish 1 marker LINE_LIST bieu dien toan bo hinh dang chu (chi de xem truoc trong RViz)
void publishPreviewMarker(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub,
  const ur3_letter_writer::Letter & strokes,
  const PlaneConfig & cfg,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "letter_preview";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.004;
  marker.color.r = 0.0;
  marker.color.g = 0.6;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0;

  for (const auto & stroke : strokes) {
    for (size_t i = 0; i + 1 < stroke.size(); ++i) {
      auto p1 = makePose(stroke[i].first, stroke[i].second, true, cfg);
      auto p2 = makePose(stroke[i + 1].first, stroke[i + 1].second, true, cfg);
      geometry_msgs::msg::Point pt1, pt2;
      pt1.x = p1.position.x; pt1.y = p1.position.y; pt1.z = p1.position.z;
      pt2.x = p2.position.x; pt2.y = p2.position.y; pt2.z = p2.position.z;
      marker.points.push_back(pt1);
      marker.points.push_back(pt2);
    }
  }
  pub->publish(marker);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("letter_writer_node");
  auto logger = node->get_logger();

  // MoveGroupInterface can node's executor dang spin de nhan service/action response
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // ---------------- Tham so cau hinh (co the truyen tu launch file) ----------------
  const std::string letter_name = node->declare_parameter<std::string>("letter_name", "Bao");
  const std::string planning_group = node->declare_parameter<std::string>("planning_group", "ur_manipulator");

  PlaneConfig cfg;
  cfg.plane_x = node->declare_parameter<double>("plane_x", 0.40);
  cfg.center_y = node->declare_parameter<double>("center_y", 0.0);
  cfg.center_z = node->declare_parameter<double>("center_z", 0.40);
  cfg.width = node->declare_parameter<double>("letter_width", 0.20);
  cfg.height = node->declare_parameter<double>("letter_height", 0.25);
  cfg.pen_lift = node->declare_parameter<double>("pen_lift", 0.05);

  const double vel_scale = node->declare_parameter<double>("velocity_scaling", 0.3);
  const double acc_scale = node->declare_parameter<double>("acceleration_scaling", 0.3);
  const double eef_step = node->declare_parameter<double>("eef_step", 0.005);

  // Vet quy dao THUC TE cua end-effector (doc qua TF trong luc robot chay)
  const bool publish_ee_trace = node->declare_parameter<bool>("publish_ee_trace", true);
  const std::string ee_trace_topic = node->declare_parameter<std::string>("ee_trace_topic", "letter_trace");
  const double ee_trace_rate = node->declare_parameter<double>("ee_trace_rate", 30.0);
  const double ee_trace_line_width = node->declare_parameter<double>("ee_trace_line_width", 0.006);
  // true  -> chi ve vet khi dau cong tac dang "cham giay" (net chu sach se)
  // false -> ve ca doan di chuyen khi nhac but (thay toan bo hanh trinh that)
  const bool trace_pen_down_only = node->declare_parameter<bool>("trace_pen_down_only", true);
  // Neu de trong, tu dong lay end-effector link cua planning group
  std::string ee_trace_frame = node->declare_parameter<std::string>("ee_trace_frame", "");

  const std::vector<double> ready_joints = node->declare_parameter<std::vector<double>>(
    "ready_joint_positions", {0.0, -1.57, 1.57, -1.57, -1.57, 0.0});
  const std::vector<double> rpy = node->declare_parameter<std::vector<double>>(
    "tool_orientation_rpy", {0.0, 1.5708, 0.0});

  tf2::Quaternion q;
  q.setRPY(rpy.at(0), rpy.at(1), rpy.at(2));
  cfg.orientation = tf2::toMsg(q);

  const char letter_char = letter_name.empty()
    ? 'A' : static_cast<char>(std::toupper(static_cast<unsigned char>(letter_name[0])));

  // ---------------- Lay danh sach net cua chu cai ----------------
  ur3_letter_writer::Letter strokes;
  try {
    strokes = ur3_letter_writer::getLetterStrokes(letter_char);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Khong the ve chu '%c': %s", letter_char, e.what());
    RCLCPP_ERROR(logger, "Cac chu duoc ho tro: %s", ur3_letter_writer::supportedLetters().c_str());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  RCLCPP_INFO(
    logger, "Se ve chu '%c' (tu ten '%s'), %zu net, kich thuoc %.0fx%.0f mm.",
    letter_char, letter_name.c_str(), strokes.size(), cfg.width * 1000.0, cfg.height * 1000.0);

  // ---------------- Khoi tao MoveGroupInterface ----------------
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setMaxVelocityScalingFactor(vel_scale);
  move_group.setMaxAccelerationScalingFactor(acc_scale);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setGoalPositionTolerance(0.002);
  move_group.setGoalOrientationTolerance(0.01);

  const std::string planning_frame = move_group.getPlanningFrame();
  if (ee_trace_frame.empty()) {
    ee_trace_frame = move_group.getEndEffectorLink();
  }
  if (ee_trace_frame.empty()) {
    ee_trace_frame = "tool0";
  }

  // ---------------- Publish marker xem truoc hinh dang chu ----------------
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "letter_preview", rclcpp::QoS(1).transient_local());
  // Depth lon hon 1: moi net chu la mot marker id rieng, RViz vao tre van
  // nhan lai duoc day du cac net da ve (transient_local giu lai lich su).
  auto trace_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    ee_trace_topic, rclcpp::QoS(50).transient_local());
  // doi mot chut de publisher ket noi voi RViz truoc khi publish (durability transient_local
  // van giup RViz nhan duoc du subscribe sau, nhung cho vai giay cho de RViz kip khoi dong)
  std::this_thread::sleep_for(1s);
  publishPreviewMarker(marker_pub, strokes, cfg, planning_frame, node->now());

  // ---------------- Thread ve vet quy dao thuc te cua end-effector ----------------
  std::unique_ptr<EndEffectorTracer> tracer;
  if (publish_ee_trace) {
    tracer = std::make_unique<EndEffectorTracer>(
      node, trace_pub, planning_frame, ee_trace_frame, ee_trace_rate, ee_trace_line_width);
    tracer->start();
    RCLCPP_INFO(
      logger, "Vet quy dao thuc te: topic '%s' (TF '%s' -> '%s'), %.1f Hz, %s.",
      ee_trace_topic.c_str(), planning_frame.c_str(), ee_trace_frame.c_str(), ee_trace_rate,
      trace_pen_down_only ? "chi ghi khi dang ve net (pen down)"
                          : "ghi toan bo hanh trinh (ke ca luc nhac but)");
  }

  // ---------------- Ve tu the san sang (tranh singularity / vi tri xuat phat an toan) ----------------
  RCLCPP_INFO(logger, "Di chuyen ve tu the san sang...");
  move_group.setJointValueTarget(ready_joints);
  move_group.move();
  // Doi mot chut de action client cua MoveGroupInterface xu ly xong ket qua
  // cua goal truoc do, tranh gui goal moi qua sat gay nham lan goal/result
  // (de xay ra hon khi CPU dang ban, vd RViz dang render cung luc).
  std::this_thread::sleep_for(200ms);

  // ---------------- Ve tung net chu ----------------
  // Che do "ghi toan bo hanh trinh": mo mot doan vet duy nhat ngay tu dau va
  // khong tam dung, nen ca cac doan nhac but chuyen net cung hien tren RViz.
  if (tracer && !trace_pen_down_only) {
    tracer->beginSegment(0);
  }

  for (size_t s = 0; s < strokes.size(); ++s) {
    const auto & stroke = strokes[s];
    if (stroke.empty()) {
      continue;
    }
    RCLCPP_INFO(logger, "Net %zu/%zu (%zu diem)", s + 1, strokes.size(), stroke.size());

    // 1) Di chuyen tu do (pen up) toi phia tren diem dau tien cua net
    auto pre_pose = makePose(stroke.front().first, stroke.front().second, false, cfg);
    move_group.setPoseTarget(pre_pose);
    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "  Khong toi duoc vi tri truoc net %zu -> bo qua net nay.", s + 1);
      continue;
    }
    std::this_thread::sleep_for(200ms);

    // 2) Ha dau cong tac xuong mat phang (pen down)
    std::vector<geometry_msgs::msg::Pose> down_wp = {
      makePose(stroke.front().first, stroke.front().second, true, cfg)};
    executeCartesianSegment(move_group, down_wp, eef_step, logger);
    std::this_thread::sleep_for(200ms);

    // 3) Ve net: di chuyen Cartesian lien tuc qua cac diem con lai, giu pen down
    //    Bat dau ghi vet ngay truoc doan nay: diem dau tien cua vet trung
    //    dung diem dau net, nen hinh ve ra chinh la net chu (khong lan sang
    //    doan ha but theo phuong vuong goc mat phang).
    if (tracer && trace_pen_down_only) {
      tracer->beginSegment(static_cast<int>(s));
    }

    std::vector<geometry_msgs::msg::Pose> stroke_wp;
    for (size_t i = 1; i < stroke.size(); ++i) {
      stroke_wp.push_back(makePose(stroke[i].first, stroke[i].second, true, cfg));
    }
    if (!stroke_wp.empty()) {
      executeCartesianSegment(move_group, stroke_wp, eef_step, logger);
      std::this_thread::sleep_for(200ms);
    }

    if (tracer && trace_pen_down_only) {
      tracer->pauseSegment();
    }

    // 4) Nhac dau cong tac len (pen up) truoc khi sang net tiep theo
    std::vector<geometry_msgs::msg::Pose> up_wp = {
      makePose(stroke.back().first, stroke.back().second, false, cfg)};
    executeCartesianSegment(move_group, up_wp, eef_step, logger);
    std::this_thread::sleep_for(200ms);
  }

  RCLCPP_INFO(logger, "Da ve xong chu '%c'. Quay ve tu the san sang.", letter_char);
  // Dung ghi vet truoc khi rut tay ve: giu nguyen hinh chu da ve tren RViz,
  // khong keo them duong tu cho ve ve tu the san sang.
  if (tracer) {
    tracer->stop();
  }
  move_group.setJointValueTarget(ready_joints);
  move_group.move();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
