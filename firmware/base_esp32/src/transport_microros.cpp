// transport_microros.cpp — ROS 2 transport via micro-ROS over native USB.
//
// TOPICS (names in config.h)
//   sub  /canbot/wheel_commands  sensor_msgs/JointState  velocity[] in rad/s, matched by joint NAME
//   pub  /canbot/wheel_states    sensor_msgs/JointState  position[] rad, velocity[] rad/s, 50 Hz
//   pub  /canbot/base_faults     std_msgs/UInt16         fault bitmask (drive_controller.h), 2 Hz
//
// These match ros2_control's topic-based hardware interface
// (joint_state_topic_hardware_interface/JointStateTopicSystem), so on the Pi the standard
// mecanum_drive_controller sees four ordinary velocity joints.
//
// QoS: both streams are BEST EFFORT. The hardware interface subscribes to states with
// SensorDataQoS (best effort), and its reliable command publisher is compatible with a
// best-effort subscriber. Best effort means no acknowledgements crossing the USB link:
// a lost command is simply replaced by the next one 10 ms later, which beats a reliable
// stream retransmitting a command that's already stale.
//
// MEMORY: micro-ROS never calls malloc while running. Every string and array a message
// can hold must be given a fixed buffer up front (bindJointState below). An incoming
// message bigger than its buffers is dropped, so the buffers are sized with slack.
//
// AGENT LIFECYCLE: the board must survive the agent starting late, restarting, or the
// cable being pulled. loop() runs a 4-state machine (the standard micro-ROS reconnection
// pattern): wait for an agent → create the node and topics → run → on loss, tear down and
// go back to waiting. While disconnected, the command watchdog has already stopped the wheels.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/u_int16.h>

#include <cstring>

#include "config.h"
#include "drive_controller.h"
#include "transport.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "env:microros expects board_microros_transport = serial"
#endif

namespace {

constexpr size_t kMaxJoints = 8;    // slack: a message naming up to 8 joints still fits
constexpr size_t kNameCapacity = 32;
constexpr size_t kFrameCapacity = 32;
constexpr uint32_t kFaultPublishEvery = 25;  // timer ticks: 25 x 20 ms = 2 Hz

// ---- Fixed storage for one JointState message ---------------------------------------
struct JointStateStorage {
  sensor_msgs__msg__JointState msg;
  rosidl_runtime_c__String names[kMaxJoints];
  char name_chars[kMaxJoints][kNameCapacity];
  char frame_chars[kFrameCapacity];
  double position[kMaxJoints];
  double velocity[kMaxJoints];
  double effort[kMaxJoints];
};

// Point every sequence in the message at the static arrays above. "capacity" is how much
// room exists; "size" is how much is in use. The deserializer fills up to capacity.
void bindJointState(JointStateStorage& s) {
  std::memset(&s.msg, 0, sizeof(s.msg));
  s.msg.header.frame_id.data = s.frame_chars;
  s.msg.header.frame_id.size = 0;
  s.msg.header.frame_id.capacity = kFrameCapacity;
  s.frame_chars[0] = '\0';
  for (size_t i = 0; i < kMaxJoints; ++i) {
    s.names[i].data = s.name_chars[i];
    s.names[i].size = 0;
    s.names[i].capacity = kNameCapacity;
    s.name_chars[i][0] = '\0';
  }
  s.msg.name.data = s.names;
  s.msg.name.size = 0;
  s.msg.name.capacity = kMaxJoints;
  s.msg.position = {s.position, 0, kMaxJoints};
  s.msg.velocity = {s.velocity, 0, kMaxJoints};
  s.msg.effort = {s.effort, 0, kMaxJoints};
}

// ---- micro-ROS entities -------------------------------------------------------------
enum class AgentState { WaitingAgent, AgentAvailable, AgentConnected, AgentDisconnected };

DriveController* g_drive = nullptr;
AgentState g_state = AgentState::WaitingAgent;

rcl_allocator_t g_allocator;
rclc_support_t g_support;
rcl_node_t g_node;
rcl_subscription_t g_cmd_sub;
rcl_publisher_t g_state_pub;
rcl_publisher_t g_fault_pub;
rcl_timer_t g_timer;
rclc_executor_t g_executor;

JointStateStorage g_cmd;    // incoming commands are deserialized into this
JointStateStorage g_out;    // outgoing states are built in this
std_msgs__msg__UInt16 g_fault_msg;
uint32_t g_timer_ticks = 0;
uint32_t g_last_ping_ms = 0;

// ---- Callbacks ------------------------------------------------------------------------

// A command is applied only if it carries a velocity for ALL four wheel joints. Accepting
// a partial message would leave the missing wheels at stale targets, and a robot whose
// wheels disagree doesn't drive straight; it spins.
void onWheelCommand(const void* msgin) {
  const auto* m = static_cast<const sensor_msgs__msg__JointState*>(msgin);
  if (m->velocity.size != m->name.size) return;

  float targets[cfg::kNumWheels] = {};
  bool found[cfg::kNumWheels] = {};
  for (size_t j = 0; j < m->name.size; ++j) {
    for (int w = 0; w < cfg::kNumWheels; ++w) {
      if (std::strncmp(m->name.data[j].data, cfg::kWheelJointName[w], kNameCapacity) == 0) {
        targets[w] = static_cast<float>(m->velocity.data[j]);
        found[w] = true;
      }
    }
  }
  for (int w = 0; w < cfg::kNumWheels; ++w) {
    if (!found[w]) return;
  }
  g_drive->commandVelocity(targets);
}

void onTimer(rcl_timer_t* timer, int64_t /*last_call_time*/) {
  if (timer == nullptr) return;
  DriveState s;
  g_drive->getState(s);

  // Time-stamp with the PC's clock (synchronized at connect), so the Pi can line wheel
  // odometry up with camera frames and other sensors that use the same clock.
  const int64_t ns = rmw_uros_epoch_nanos();
  g_out.msg.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  g_out.msg.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  for (int w = 0; w < cfg::kNumWheels; ++w) {
    g_out.position[w] = s.position_rad[w];
    g_out.velocity[w] = s.velocity_rad_s[w];
  }
  rcl_ret_t rc = rcl_publish(&g_state_pub, &g_out.msg, nullptr);
  (void)rc;  // best effort: a dropped sample is replaced 20 ms later

  if (++g_timer_ticks % kFaultPublishEvery == 0) {
    g_fault_msg.data = s.faults;
    rc = rcl_publish(&g_fault_pub, &g_fault_msg, nullptr);
    (void)rc;
  }
}

// ---- Entity lifecycle -----------------------------------------------------------------

bool createEntities() {
  // Start every handle from a known-empty state. If creation fails halfway,
  // destroyEntities() then finalizes only what was actually created.
  std::memset(&g_support, 0, sizeof(g_support));
  g_node = rcl_get_zero_initialized_node();
  g_cmd_sub = rcl_get_zero_initialized_subscription();
  g_state_pub = rcl_get_zero_initialized_publisher();
  g_fault_pub = rcl_get_zero_initialized_publisher();
  g_timer = rcl_get_zero_initialized_timer();
  g_executor = rclc_executor_get_zero_initialized_executor();

  g_allocator = rcl_get_default_allocator();
  if (rclc_support_init(&g_support, 0, nullptr, &g_allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&g_node, cfg::kNodeName, "", &g_support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_best_effort(&g_cmd_sub, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                         cfg::kWheelCommandTopic) != RCL_RET_OK)
    return false;
  if (rclc_publisher_init_best_effort(&g_state_pub, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                      cfg::kWheelStateTopic) != RCL_RET_OK)
    return false;
  if (rclc_publisher_init_best_effort(&g_fault_pub, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
                                      cfg::kFaultTopic) != RCL_RET_OK)
    return false;
  if (rclc_timer_init_default2(&g_timer, &g_support, RCL_MS_TO_NS(cfg::kStatePublishPeriodMs), onTimer, true) !=
      RCL_RET_OK)
    return false;

  // Two handles: the command subscription and the publish timer.
  if (rclc_executor_init(&g_executor, &g_support.context, 2, &g_allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&g_executor, &g_cmd_sub, &g_cmd.msg, &onWheelCommand, ON_NEW_DATA) != RCL_RET_OK)
    return false;
  if (rclc_executor_add_timer(&g_executor, &g_timer) != RCL_RET_OK) return false;

  // Align the board's clock with the agent's (NTP-style). Retried every reconnect.
  rmw_uros_sync_session(100);
  return true;
}

void destroyEntities() {
  // The agent is probably gone, so don't wait for it to acknowledge each deletion.
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&g_support.context);
  if (rmw_context != nullptr) (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc;
  rc = rcl_subscription_fini(&g_cmd_sub, &g_node);
  rc = rcl_publisher_fini(&g_state_pub, &g_node);
  rc = rcl_publisher_fini(&g_fault_pub, &g_node);
  rc = rcl_timer_fini(&g_timer);
  rc = rclc_executor_fini(&g_executor);
  rc = rcl_node_fini(&g_node);
  rc = rclc_support_fini(&g_support);
  (void)rc;
  g_drive->stop();  // don't wait for the watchdog: we know the host is gone
}

}  // namespace

namespace transport {

void begin(DriveController& drive) {
  g_drive = &drive;
  bindJointState(g_cmd);
  bindJointState(g_out);

  // The outgoing message always names the same four joints, so fill the names once.
  for (int w = 0; w < cfg::kNumWheels; ++w) {
    std::strncpy(g_out.name_chars[w], cfg::kWheelJointName[w], kNameCapacity - 1);
    g_out.name_chars[w][kNameCapacity - 1] = '\0';
    g_out.names[w].size = std::strlen(g_out.name_chars[w]);
  }
  g_out.msg.name.size = cfg::kNumWheels;
  g_out.msg.position.size = cfg::kNumWheels;
  g_out.msg.velocity.size = cfg::kNumWheels;
  g_out.msg.effort.size = 0;  // no torque sensing on these motors

  // Serial is the S3's native USB port (ARDUINO_USB_CDC_ON_BOOT=1 in platformio.ini).
  // Nothing else may print to it in this build: the agent would read the text as
  // corrupt protocol frames.
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  g_state = AgentState::WaitingAgent;
}

void spin() {
  const uint32_t now = millis();
  switch (g_state) {
    case AgentState::WaitingAgent:
      if (now - g_last_ping_ms >= 500) {
        g_last_ping_ms = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) g_state = AgentState::AgentAvailable;
      }
      break;

    case AgentState::AgentAvailable:
      if (createEntities()) {
        g_state = AgentState::AgentConnected;
      } else {
        destroyEntities();
        g_state = AgentState::WaitingAgent;
      }
      break;

    case AgentState::AgentConnected:
      if (now - g_last_ping_ms >= 200) {
        g_last_ping_ms = now;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          g_state = AgentState::AgentDisconnected;
          break;
        }
      }
      rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));
      break;

    case AgentState::AgentDisconnected:
      destroyEntities();
      g_state = AgentState::WaitingAgent;
      break;
  }
}

}  // namespace transport
