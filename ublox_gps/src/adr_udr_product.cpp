#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/esf_ins.hpp>
#include <ublox_msgs/msg/esf_meas.hpp>
#include <ublox_msgs/msg/esf_raw.hpp>
#include <ublox_msgs/msg/esf_status.hpp>
#include <ublox_msgs/msg/hnr_pvt.hpp>
#include <ublox_msgs/msg/nav_att.hpp>

#include <ublox_gps/adr_udr_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// u-blox ADR devices, partially implemented
//
AdrUdrProduct::AdrUdrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, rclcpp::Node* node)
  : use_adr_(false), nav_rate_(nav_rate), meas_rate_(meas_rate), frame_id_(frame_id), updater_(updater), node_(node)
{
  if (getRosBoolean(node_, "publish.esf.meas")) {
    imu_pub_ =
      node_->create_publisher<sensor_msgs::msg::Imu>("imu_meas", 1);
    time_ref_pub_ =
      node_->create_publisher<sensor_msgs::msg::TimeReference>("interrupt_time", 1);

    esf_meas_pub_ = node_->create_publisher<ublox_msgs::msg::EsfMEAS>("esfmeas", 1);
  }
  if (getRosBoolean(node_, "publish.nav.att")) {
    nav_att_pub_ = node_->create_publisher<ublox_msgs::msg::NavATT>("navatt", 1);
  }
  if (getRosBoolean(node_, "publish.esf.alg")) {
    esf_alg_pub_ = node_->create_publisher<ublox_msgs::msg::EsfALG>("esfalg", 1);
  }
  if (getRosBoolean(node_, "publish.esf.ins")) {
    esf_ins_pub_ = node_->create_publisher<ublox_msgs::msg::EsfINS>("esfins", 1);
  }
  if (getRosBoolean(node_, "publish.esf.raw")) {
    esf_raw_pub_ = node_->create_publisher<ublox_msgs::msg::EsfRAW>("esfraw", 1);
  }
  if (getRosBoolean(node_, "publish.esf.status")) {
    esf_status_pub_ = node_->create_publisher<ublox_msgs::msg::EsfSTATUS>("esfstatus", 1);
  }
  if (getRosBoolean(node_, "publish.hnr.pvt")) {
    hnr_pvt_pub_ = node_->create_publisher<ublox_msgs::msg::HnrPVT>("hnrpvt", 1);
  }
}

void AdrUdrProduct::getRosParams() {
  use_adr_ = getRosBoolean(node_, "use_adr");
  // Check the nav rate
  float nav_rate_hz = 1000.0 / (meas_rate_ * nav_rate_);
  if (nav_rate_hz != 1) {
    RCLCPP_WARN(node_->get_logger(), "Nav Rate recommended to be 1 Hz");
  }
}

bool AdrUdrProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  this->gps_ = gps;
  if (!gps->setUseAdr(use_adr_)) {
    throw std::runtime_error(std::string("Failed to ")
                             + (use_adr_ ? "enable" : "disable") + "use_adr");
  }
  return true;
}

void AdrUdrProduct::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const int32_t sec = msg->header.stamp.sec;
  const uint32_t nanosec = msg->header.stamp.nanosec;
  const int64_t millis = (sec * 1e3 + nanosec * 1e-6) - time_tag_delta_;
  const uint32_t ttag = static_cast<uint32_t>(millis);
  gps_->sendLinearVelocity(msg->twist.twist.linear.x, ttag);
}

void AdrUdrProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to NAV ATT messages
  if (getRosBoolean(node_, "publish.nav.att")) {
    gps->subscribe<ublox_msgs::msg::NavATT>([this](const ublox_msgs::msg::NavATT &m) { nav_att_pub_->publish(m); },
                                       1);
  }

  // Subscribe to ESF ALG messages
  if (getRosBoolean(node_, "publish.esf.alg")) {
    gps->subscribe<ublox_msgs::msg::EsfALG>([this](const ublox_msgs::msg::EsfALG &m) { esf_alg_pub_->publish(m); },
                                       1);
  }

  // Subscribe to ESF INS messages
  if (getRosBoolean(node_, "publish.esf.ins")) {
    gps->subscribe<ublox_msgs::msg::EsfINS>([this](const ublox_msgs::msg::EsfINS &m) { esf_ins_pub_->publish(m); },
                                       1);
  }

  // Subscribe to ESF Meas messages
  if (getRosBoolean(node_, "publish.esf.meas")) {
    gps->subscribe<ublox_msgs::msg::EsfMEAS>([this](const ublox_msgs::msg::EsfMEAS &m) { esf_meas_pub_->publish(m); },
                                        1);

    // also publish sensor_msgs::Imu
    gps->subscribe<ublox_msgs::msg::EsfMEAS>(std::bind(
      &AdrUdrProduct::callbackEsfMEAS, this, std::placeholders::_1), 1);
  }

  // Subscribe to ESF Raw messages
  if (getRosBoolean(node_, "publish.esf.raw")) {
    gps->subscribe<ublox_msgs::msg::EsfRAW>([this](const ublox_msgs::msg::EsfRAW &m) { esf_raw_pub_->publish(m); },
                                       1);
  }

  // Subscribe to ESF Status messages
  if (getRosBoolean(node_, "publish.esf.status")) {
    gps->subscribe<ublox_msgs::msg::EsfSTATUS>([this](const ublox_msgs::msg::EsfSTATUS &m) { esf_status_pub_->publish(m); },
                                          1);
  }

  // Subscribe to HNR PVT messages
  if (getRosBoolean(node_, "publish.hnr.pvt")) {
    gps->subscribe<ublox_msgs::msg::HnrPVT>([this](const ublox_msgs::msg::HnrPVT &m) { hnr_pvt_pub_->publish(m); },
                                       1);
  }

  if (getRosBoolean(node_, "subscribe.esf.odom")) {
    this->subscription_ =
      node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        10,
        std::bind(&AdrUdrProduct::odomCallback, this, std::placeholders::_1));
  }
}

void AdrUdrProduct::callbackEsfMEAS(const ublox_msgs::msg::EsfMEAS &m) {
  // time_tag_delta_ = imu_.header.stamp - m.time_tag;
  // RCLCPP_WARN_STREAM(node_->get_logger(), "Current time delta: " << time_tag_delta_);
  // RCLCPP_WARN_STREAM(node_->get_logger(), "msg: " << m);

  if (getRosBoolean(node_, "publish.esf.meas")) {
    const auto sys_time = node_->now();
    const uint64_t nanoseconds = sys_time.nanoseconds();
    const uint64_t millis = nanoseconds * 1e-6;
    const uint32_t millis_truncated = millis & 0xFFFFFFFF;
    time_tag_delta_ = millis_truncated - m.time_tag;

    imu_.header.stamp = sys_time;
    imu_.header.frame_id = frame_id_;

    float rad_per_sec = ::pow(2, -12) * M_PI / 180.0F;
    float m_per_sec_sq = ::pow(2, -10);

    std::vector<unsigned int> imu_data = m.data;
    for (unsigned int datapoint : imu_data) {
      unsigned int data_type = datapoint >> 24;  // grab the last six bits of data
      int32_t data_value = static_cast<int32_t>(datapoint << 8);
      data_value >>= 8;  // carries the sign correctly

      imu_.orientation_covariance[0] = -1;
      imu_.linear_acceleration_covariance[0] = -1;
      imu_.angular_velocity_covariance[0] = -1;

      if (data_type == 14) {
        imu_.angular_velocity.x = data_value * rad_per_sec;
        imu_rec_flag_ |= 0b00000001;
      } else if (data_type == 16) {
        imu_.linear_acceleration.x = data_value * m_per_sec_sq;
        imu_rec_flag_ |= 0b00000010;
      } else if (data_type == 13) {
        imu_.angular_velocity.y = data_value * rad_per_sec;
        imu_rec_flag_ |= 0b00000100;
      } else if (data_type == 17) {
        imu_.linear_acceleration.y = data_value * m_per_sec_sq;
        imu_rec_flag_ |= 0b00001000;
      } else if (data_type == 5) {
        imu_.angular_velocity.z = data_value * rad_per_sec;
        imu_rec_flag_ |= 0b00010000;
      } else if (data_type == 18) {
        imu_.linear_acceleration.z = data_value * m_per_sec_sq;
        imu_rec_flag_ |= 0b00100000;
      } else if (data_type == 12) {
        // RCLCPP_INFO("Temperature in celsius: %f", data_value * deg_c);
      } else {
        RCLCPP_INFO(node_->get_logger(), "data_type: %u", data_type);
        RCLCPP_INFO(node_->get_logger(), "data_value: %u", data_value);
      }
    }

    if (imu_rec_flag_ == 0b00111111) {
      imu_pub_->publish(imu_);
      imu_rec_flag_ = 0;
    }
  }
}

}  // namespace ublox_node
