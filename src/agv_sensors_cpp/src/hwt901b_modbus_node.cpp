#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{
constexpr double kGravity = 9.80665;

uint16_t crc16_modbus(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

std::vector<uint8_t> build_read_request(uint8_t slave, uint16_t start_addr, uint16_t count)
{
  std::vector<uint8_t> req = {
    slave,
    0x03,
    static_cast<uint8_t>((start_addr >> 8) & 0xFF),
    static_cast<uint8_t>(start_addr & 0xFF),
    static_cast<uint8_t>((count >> 8) & 0xFF),
    static_cast<uint8_t>(count & 0xFF),
  };
  const uint16_t crc = crc16_modbus(req.data(), req.size());
  req.push_back(static_cast<uint8_t>(crc & 0xFF));
  req.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return req;
}

speed_t baud_to_termios(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

bool parse_registers(const std::vector<uint8_t> & resp, int expected_count, std::vector<int16_t> & regs)
{
  if (resp.size() < 5) {
    return false;
  }
  const uint16_t rx_crc = static_cast<uint16_t>(resp[resp.size() - 2] |
    (resp[resp.size() - 1] << 8));
  const uint16_t calc_crc = crc16_modbus(resp.data(), resp.size() - 2);
  if (rx_crc != calc_crc || resp[1] != 0x03) {
    return false;
  }
  const int byte_count = resp[2];
  if (byte_count != expected_count * 2 || resp.size() != static_cast<size_t>(5 + byte_count)) {
    return false;
  }

  regs.clear();
  regs.reserve(expected_count);
  for (int i = 0; i < byte_count; i += 2) {
    const uint16_t raw = static_cast<uint16_t>((resp[3 + i] << 8) | resp[3 + i + 1]);
    regs.push_back(static_cast<int16_t>(raw));
  }
  return true;
}

std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  };
}
}  // namespace

class HWT901BModbusNode : public rclcpp::Node
{
public:
  HWT901BModbusNode()
  : Node("hwt901b_modbus"), fd_(-1)
  {
    port_ = declare_parameter<std::string>(
      "port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
    baud_ = declare_parameter<int>("baud", 115200);
    slave_id_ = static_cast<uint8_t>(declare_parameter<int>("slave_id", 0x51));
    poll_hz_ = declare_parameter<double>("poll_hz", 100.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    orientation_topic_ = declare_parameter<std::string>("orientation_topic", "/imu");
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/imu/data_raw");
    // Backward-compatible alias used by older launch files. Treat it as the raw topic.
    raw_topic_ = declare_parameter<std::string>("topic", raw_topic_);
    orientation_rate_ = declare_parameter<double>("orientation_rate", 50.0);
    raw_rate_ = declare_parameter<double>("raw_rate", 100.0);
    time_offset_sec_ = declare_parameter<double>("time_offset_sec", 0.0);
    // Kept for old launch files. The orientation topic now always publishes a valid quaternion.
    publish_orientation_ = declare_parameter<bool>("publish_orientation", false);
    read_timeout_ms_ = declare_parameter<int>("read_timeout_ms", 10);
    write_timeout_ms_ = declare_parameter<int>("write_timeout_ms", 5);
    acc_scale_g_ = declare_parameter<double>("acc_scale_g", 16.0);
    gyro_scale_dps_ = declare_parameter<double>("gyro_scale_dps", 2000.0);
    // Calibrated stationary on 2026-06-28. Orientation uses conservative
    // floors because static variance does not include magnetic disturbances.
    orientation_variance_roll_ =
      declare_parameter<double>("orientation_variance_roll", 0.000076152);
    orientation_variance_pitch_ =
      declare_parameter<double>("orientation_variance_pitch", 0.000076152);
    orientation_variance_yaw_ =
      declare_parameter<double>("orientation_variance_yaw", 0.000352581);
    raw_gyro_variance_x_ =
      declare_parameter<double>("raw_gyro_variance_x", 2.389346e-7);
    raw_gyro_variance_y_ =
      declare_parameter<double>("raw_gyro_variance_y", 3.498908e-7);
    raw_gyro_variance_z_ =
      declare_parameter<double>("raw_gyro_variance_z", 1.0e-7);
    raw_accel_variance_x_ =
      declare_parameter<double>("raw_accel_variance_x", 2.642381e-5);
    raw_accel_variance_y_ =
      declare_parameter<double>("raw_accel_variance_y", 4.059536e-5);
    raw_accel_variance_z_ =
      declare_parameter<double>("raw_accel_variance_z", 3.055125e-4);

    raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(raw_topic_, 100);
    orientation_pub_ = create_publisher<sensor_msgs::msg::Imu>(orientation_topic_, 50);
    request_ = build_read_request(slave_id_, 0x0034, register_count_);
    expected_len_ = static_cast<size_t>(5 + 2 * register_count_);

    acquire_lock();
    open_serial();

    const double sensor_hz = std::max(1.0, std::max(poll_hz_, raw_rate_));
    orientation_decimation_ = std::max(
      1, static_cast<int>(std::round(sensor_hz / std::max(1.0, orientation_rate_))));
    const auto period = std::chrono::duration<double>(1.0 / sensor_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&HWT901BModbusNode::poll_once, this));

    RCLCPP_INFO(
      get_logger(),
      "HWT901B C++ Modbus IMU: port=%s baud=%d slave=0x%02X poll=%.1fHz raw=%s@%.1fHz orientation=%s@%.1fHz frame=%s time_offset=%.6fs read_timeout=%dms write_timeout=%dms",
      port_.c_str(), baud_, slave_id_, sensor_hz, raw_topic_.c_str(), raw_rate_,
      orientation_topic_.c_str(), orientation_rate_, frame_id_.c_str(), time_offset_sec_,
      read_timeout_ms_, write_timeout_ms_);
    RCLCPP_INFO(
      get_logger(),
      "HWT901B covariance: orientation diag=[%.9g %.9g %.9g] rad^2, gyro diag=[%.9g %.9g %.9g], accel diag=[%.9g %.9g %.9g]",
      orientation_variance_roll_, orientation_variance_pitch_, orientation_variance_yaw_,
      raw_gyro_variance_x_, raw_gyro_variance_y_, raw_gyro_variance_z_,
      raw_accel_variance_x_, raw_accel_variance_y_, raw_accel_variance_z_);
  }

  ~HWT901BModbusNode() override
  {
    close_serial();
    release_lock();
  }

private:
  void acquire_lock()
  {
    lock_fd_ = open("/tmp/amr_imu_serial.lock", O_CREAT | O_RDWR, 0666);
    if (lock_fd_ < 0) {
      throw std::runtime_error("Cannot open IMU serial lock: " + std::string(std::strerror(errno)));
    }
    if (flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      throw std::runtime_error(
        "IMU serial is already locked by another process. Stop duplicate IMU nodes first.");
    }
  }

  void release_lock()
  {
    if (lock_fd_ >= 0) {
      flock(lock_fd_, LOCK_UN);
      close(lock_fd_);
      lock_fd_ = -1;
    }
  }

  bool write_all(const uint8_t * data, size_t size)
  {
    size_t sent = 0;
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(1, write_timeout_ms_));
    while (sent < size && std::chrono::steady_clock::now() < deadline) {
      const ssize_t n = write(fd_, data + sent, size - sent);
      if (n > 0) {
        sent += static_cast<size_t>(n);
        continue;
      }
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        return false;
      }
      pollfd pfd {fd_, POLLOUT, 0};
      (void)poll(&pfd, 1, 2);
    }
    return sent == size;
  }

  void close_serial()
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  void open_serial()
  {
    close_serial();
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Cannot open IMU serial port: " + port_ + ": " + std::strerror(errno));
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error("tcgetattr failed for IMU serial port");
    }

    cfmakeraw(&tty);
    const speed_t speed = baud_to_termios(baud_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("tcsetattr failed for IMU serial port");
    }
    tcflush(fd_, TCIOFLUSH);
  }

  void poll_once()
  {
    if (fd_ < 0) {
      try {
        open_serial();
        consecutive_errors_ = 0;
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "IMU reconnect failed: %s", e.what());
      }
      return;
    }

    tcflush(fd_, TCIFLUSH);
    const rclcpp::Time t_request = this->get_clock()->now();
    if (!write_all(request_.data(), request_.size())) {
      ++consecutive_errors_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU serial write failed");
      if (consecutive_errors_ >= max_consecutive_errors_) {
        RCLCPP_WARN(get_logger(), "IMU serial unhealthy; reopening port");
        close_serial();
        consecutive_errors_ = 0;
      }
      return;
    }
    tcdrain(fd_);

    std::vector<uint8_t> resp(expected_len_, 0);
    size_t got = 0;
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(1, read_timeout_ms_));
    while (got < expected_len_ && std::chrono::steady_clock::now() < deadline) {
      const ssize_t n = read(fd_, resp.data() + got, expected_len_ - got);
      if (n > 0) {
        got += static_cast<size_t>(n);
      } else {
        pollfd pfd {fd_, POLLIN, 0};
        (void)poll(&pfd, 1, 2);
      }
    }
    resp.resize(got);

    std::vector<int16_t> regs;
    if (!parse_registers(resp, register_count_, regs)) {
      ++consecutive_errors_;
      RCLCPP_DEBUG(get_logger(), "Invalid IMU Modbus response");
      if (consecutive_errors_ >= max_consecutive_errors_) {
        RCLCPP_WARN(get_logger(), "IMU response timeout/CRC repeated; reopening port");
        close_serial();
        consecutive_errors_ = 0;
      }
      return;
    }
    const rclcpp::Time t_response = this->get_clock()->now();

    const int64_t midpoint_ns =
      (t_request.nanoseconds() + t_response.nanoseconds()) / 2 +
      static_cast<int64_t>(time_offset_sec_ * 1e9);
    const rclcpp::Time stamp(midpoint_ns, t_request.get_clock_type());

    consecutive_errors_ = 0;
    publish_imu(regs, stamp);
  }

  void publish_imu(const std::vector<int16_t> & regs, const rclcpp::Time & stamp)
  {
    const double acc_scale = (acc_scale_g_ * kGravity) / 32768.0;
    const double gyro_scale = (gyro_scale_dps_ * M_PI / 180.0) / 32768.0;

    const double ax = regs[0] * acc_scale;
    const double ay = regs[1] * acc_scale;
    const double az = regs[2] * acc_scale;
    const double gx = regs[3] * gyro_scale;
    const double gy = regs[4] * gyro_scale;
    const double gz = regs[5] * gyro_scale;
    const double roll = regs[9] / 32768.0 * M_PI;
    const double pitch = regs[10] / 32768.0 * M_PI;
    const double yaw = regs[11] / 32768.0 * M_PI;

    sensor_msgs::msg::Imu raw_msg;
    raw_msg.header.stamp = stamp;
    raw_msg.header.frame_id = frame_id_;
    raw_msg.linear_acceleration.x = ax;
    raw_msg.linear_acceleration.y = ay;
    raw_msg.linear_acceleration.z = az;
    raw_msg.angular_velocity.x = gx;
    raw_msg.angular_velocity.y = gy;
    raw_msg.angular_velocity.z = gz;
    raw_msg.orientation.x = 0.0;
    raw_msg.orientation.y = 0.0;
    raw_msg.orientation.z = 0.0;
    raw_msg.orientation.w = 1.0;
    raw_msg.orientation_covariance = {
      -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0};
    raw_msg.linear_acceleration_covariance = {
      raw_accel_variance_x_, 0.0, 0.0,
      0.0, raw_accel_variance_y_, 0.0,
      0.0, 0.0, raw_accel_variance_z_};
    raw_msg.angular_velocity_covariance = {
      raw_gyro_variance_x_, 0.0, 0.0,
      0.0, raw_gyro_variance_y_, 0.0,
      0.0, 0.0, raw_gyro_variance_z_};

    raw_pub_->publish(raw_msg);

    ++sample_count_;
    if (sample_count_ % orientation_decimation_ != 0) {
      return;
    }

    sensor_msgs::msg::Imu orientation_msg = raw_msg;
    const auto q = euler_to_quaternion(roll, pitch, yaw);
    orientation_msg.orientation.x = q[0];
    orientation_msg.orientation.y = q[1];
    orientation_msg.orientation.z = q[2];
    orientation_msg.orientation.w = q[3];
    orientation_msg.orientation_covariance = {
      orientation_variance_roll_, 0.0, 0.0,
      0.0, orientation_variance_pitch_, 0.0,
      0.0, 0.0, orientation_variance_yaw_};

    // Keep /imu strictly orientation-only. Raw gyro and acceleration stay on
    // /imu/data_raw for consumers such as ORB-SLAM3 RGB-D-Inertial.
    orientation_msg.angular_velocity.x = 0.0;
    orientation_msg.angular_velocity.y = 0.0;
    orientation_msg.angular_velocity.z = 0.0;
    orientation_msg.angular_velocity_covariance = {
      -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0};
    orientation_msg.linear_acceleration.x = 0.0;
    orientation_msg.linear_acceleration.y = 0.0;
    orientation_msg.linear_acceleration.z = 0.0;
    orientation_msg.linear_acceleration_covariance = {
      -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0};
    orientation_pub_->publish(orientation_msg);
  }

  static constexpr int register_count_ = 12;
  int fd_;
  int lock_fd_ {-1};
  std::string port_;
  int baud_;
  uint8_t slave_id_;
  double poll_hz_;
  std::string frame_id_;
  std::string orientation_topic_;
  std::string raw_topic_;
  double orientation_rate_;
  double raw_rate_;
  double time_offset_sec_;
  bool publish_orientation_;
  int read_timeout_ms_;
  int write_timeout_ms_;
  double acc_scale_g_;
  double gyro_scale_dps_;
  double orientation_variance_roll_;
  double orientation_variance_pitch_;
  double orientation_variance_yaw_;
  double raw_gyro_variance_x_;
  double raw_gyro_variance_y_;
  double raw_gyro_variance_z_;
  double raw_accel_variance_x_;
  double raw_accel_variance_y_;
  double raw_accel_variance_z_;
  size_t expected_len_;
  std::vector<uint8_t> request_;
  int consecutive_errors_ {0};
  int orientation_decimation_ {2};
  uint64_t sample_count_ {0};
  static constexpr int max_consecutive_errors_ = 100;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr orientation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HWT901BModbusNode>());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "hwt901b_modbus_node failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
