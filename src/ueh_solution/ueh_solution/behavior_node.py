#!/usr/bin/env python3
"""Behavior FSM node for UEH CRC 2026.

This is the MASTER node. It subscribes to all perception outputs and
produces /cmd_vel commands according to a strict priority hierarchy.

Permitted sensor subscriptions (direct):
  /scan              – used for STOP-sign approach distance
  /odom              – used for short-term motion estimation in state timers

Perception subscriptions (from our own perception nodes):
  /lane/error         (Float32)  – lane centre error in pixels
  /lane/dark_mode     (Int32)    – 1 if dark/tunnel scene
  /obstacle/front_dist (Float32) – LiDAR front distance
  /obstacle/status    (Int32)    – 0=clear, 1=slow, 2=stop
  /sign/detection     (String)   – 'STOP' | 'crosswalk' | 'none' | ...
  /light/state        (String)   – 'RED' | 'YELLOW' | 'GREEN' | 'NONE'
  /pedestrian/blocking (Bool)    – True if pedestrian in road

Publishes:
  /cmd_vel           (geometry_msgs/Twist)

FORBIDDEN (never subscribed here):
  /traffic_lights  /traffic_light/*  /automobile/semaphores
  /sky_cam/*  /model_states  /link_states
  /get_entity_state  /set_entity_state  /spawn_entity

State priority (higher number = lower priority):
  0  EMERGENCY_STOP       LiDAR obstacle < emergency_stop_dist
  1  PEDESTRIAN_STOP      Pedestrian blocking road
  2  RED_LIGHT_STOP       Traffic light RED or YELLOW
  3  STOP_SIGN_DETECTED   Approaching STOP sign
  4  STOP_SIGN_BRAKE      Slowing to stop line
  5  STOP_SIGN_HOLD       Holding complete stop for 2+ seconds
  6  STOP_SIGN_COOLDOWN   Ignoring that sign for N seconds
  7  LANE_FOLLOWING       Normal driving (lowest priority, highest throughput)

The STOP sign FSM is embedded inside state 3-6.
"""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String

# ---- FSM State IDs ---------------------------------------------------------
ST_LANE             = 'LANE_FOLLOWING'
ST_STOP_DETECTED    = 'STOP_DETECTED'
ST_STOP_BRAKE       = 'STOP_BRAKE'
ST_STOP_HOLD        = 'STOP_HOLD'
ST_STOP_COOLDOWN    = 'STOP_COOLDOWN'
ST_RED_LIGHT        = 'RED_LIGHT_STOP'
ST_PEDESTRIAN       = 'PEDESTRIAN_STOP'
ST_EMERGENCY        = 'EMERGENCY_STOP'
ST_OVERTAKE_CONFIRM = 'OVERTAKE_CONFIRM'
ST_OVERTAKE_CHANGE  = 'OVERTAKE_CHANGE'
ST_OVERTAKE_PASS    = 'OVERTAKE_PASS'
ST_OVERTAKE_RETURN  = 'OVERTAKE_RETURN'


def catch_sigterm():
    stopping = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.update(now=True))
    return stopping


class BehaviorNode(Node):

    def __init__(self):
        super().__init__('behavior_node')

        # ---- Parameters (all in params.yaml) --------------------------------
        self.declare_parameter('base_speed',                0.18)
        self.declare_parameter('max_speed',                 0.22)
        self.declare_parameter('min_speed',                 0.06)
        self.declare_parameter('kp_steer',                  0.0042)
        self.declare_parameter('kd_steer',                  0.0008)
        self.declare_parameter('max_angular_velocity',      2.0)
        self.declare_parameter('steer_slowdown_threshold',  60)
        self.declare_parameter('steer_slowdown_factor',     0.55)
        self.declare_parameter('emergency_stop_dist',       0.25)
        self.declare_parameter('slow_down_dist',            0.55)
        self.declare_parameter('slow_down_factor',          0.50)
        self.declare_parameter('stop_hold_seconds',         2.2)
        self.declare_parameter('stop_cooldown_secs',        5.0)
        self.declare_parameter('stop_brake_dist',           0.45)
        self.declare_parameter('overtaking_enabled',        False)
        self.declare_parameter('overtake_confirm_secs',     3.0)
        self.declare_parameter('overtake_lateral_w',        0.65)
        self.declare_parameter('overtake_pass_time',        4.0)
        self.declare_parameter('overtake_return_w',        -0.55)
        self.declare_parameter('enable_pedestrian',        True)
        self.declare_parameter('test_mode',                '')
        self.declare_parameter('rate',                     20.0)

        # Read all params
        def p(name):
            return self.get_parameter(name).value

        self.base_speed        = float(p('base_speed'))
        self.max_speed         = float(p('max_speed'))
        self.min_speed         = float(p('min_speed'))
        self.kp                = float(p('kp_steer'))
        self.kd                = float(p('kd_steer'))
        self.max_w             = float(p('max_angular_velocity'))
        self.slow_thr          = float(p('steer_slowdown_threshold'))
        self.slow_factor       = float(p('steer_slowdown_factor'))
        self.estop_dist        = float(p('emergency_stop_dist'))
        self.slow_dist         = float(p('slow_down_dist'))
        self.slow_speed_factor = float(p('slow_down_factor'))
        self.stop_hold_s       = float(p('stop_hold_seconds'))
        self.stop_cooldown_s   = float(p('stop_cooldown_secs'))
        self.stop_brake_dist   = float(p('stop_brake_dist'))
        self.overtaking_on     = bool(p('overtaking_enabled'))
        self.overtake_conf_s   = float(p('overtake_confirm_secs'))
        self.overtake_lat_w    = float(p('overtake_lateral_w'))
        self.overtake_pass_t   = float(p('overtake_pass_time'))
        self.overtake_ret_w    = float(p('overtake_return_w'))
        self.enable_pedestrian = bool(p('enable_pedestrian'))
        self.test_mode         = str(p('test_mode')).strip()

        # ---- Sensor / Perception state variables ----------------------------
        self.lane_error      = 0.0
        self.dark_mode       = 0
        self.front_dist      = float('inf')
        self.lidar_status    = 0          # 0=clear,1=slow,2=stop
        self.sign_detected   = 'none'
        self.light_state     = 'NONE'
        self.ped_blocking    = False
        self.prev_lane_error = 0.0

        # ---- FSM state ------------------------------------------------------
        self.state      = ST_LANE
        self.state_t0   = time.time()     # wall-clock time of last state change

        # STOP sign sub-state timers
        self._stop_sign_count = 0         # how many unique STOP signs handled

        # Overtaking sub-state
        self._overtake_conf_t0 = None

        # ---- Publishers / Subscribers ---------------------------------------
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(
            Float32, '/lane/error',
            lambda m: setattr(self, 'lane_error', m.data), 10)
        self.create_subscription(
            Int32, '/lane/dark_mode',
            lambda m: setattr(self, 'dark_mode', m.data), 10)
        self.create_subscription(
            Float32, '/obstacle/front_dist',
            lambda m: setattr(self, 'front_dist', m.data), 10)
        self.create_subscription(
            Int32, '/obstacle/status',
            lambda m: setattr(self, 'lidar_status', m.data), 10)
        self.create_subscription(
            String, '/sign/detection',
            lambda m: setattr(self, 'sign_detected', m.data), 10)
        self.create_subscription(
            String, '/light/state',
            lambda m: setattr(self, 'light_state', m.data), 10)
        self.create_subscription(
            Bool, '/pedestrian/blocking',
            lambda m: setattr(self, 'ped_blocking', m.data), 10)

        rate = float(p('rate'))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'behavior_node ready | state={self.state} | mode={self.test_mode or "normal"} | '
            f'ped_enabled={self.enable_pedestrian} | base_speed={self.base_speed:.2f} m/s')

    # ======================================================================== #
    # MAIN CONTROL LOOP
    # ======================================================================== #
    def _tick(self):
        try:
            self._fsm_step()
        except Exception as e:
            self.get_logger().error(f'behavior _tick raised: {e}')
            self._publish(0.0, 0.0)

    def _fsm_step(self):
        now = time.time()
        dt  = now - self.state_t0

        # -------------------------------------------------------------------- #
        # PRIORITY 0 – EMERGENCY STOP (LiDAR imminent collision)
        # -------------------------------------------------------------------- #
        if self.front_dist <= self.estop_dist:
            if self.state != ST_EMERGENCY:
                self._transition(ST_EMERGENCY)
            self._publish(0.0, 0.0)
            self._log_rate(2.0, f'EMERGENCY_STOP front={self.front_dist:.2f}m')
            return

        # If we were in EMERGENCY and obstacle cleared, resume
        if self.state == ST_EMERGENCY:
            self._transition(ST_LANE)

        # -------------------------------------------------------------------- #
        # GLOBAL TEST MODE: lane_only
        # In lane_only mode:
        # - lane perception enabled
        # - lane controller enabled
        # - simple LiDAR emergency safety remains enabled (Priority 0 above)
        # - pedestrian behavior disabled
        # - sign behavior disabled
        # - traffic-light behavior disabled
        # -------------------------------------------------------------------- #
        if self.test_mode == 'lane_only':
            self._lane_following_step()
            return

        # -------------------------------------------------------------------- #
        # PRIORITY 1 – PEDESTRIAN STOP
        # -------------------------------------------------------------------- #
        if self.enable_pedestrian and self.ped_blocking:
            if self.state != ST_PEDESTRIAN:
                self._transition(ST_PEDESTRIAN)
                self.get_logger().info('PEDESTRIAN: stopping')
            self._publish(0.0, 0.0)
            return

        if self.state == ST_PEDESTRIAN:
            self.get_logger().info('PEDESTRIAN: cleared, resuming')
            self._transition(ST_LANE)

        # -------------------------------------------------------------------- #
        # PRIORITY 2 – TRAFFIC LIGHT (RED or YELLOW → stop)
        # -------------------------------------------------------------------- #
        if self.light_state in ('RED', 'YELLOW'):
            if self.state != ST_RED_LIGHT:
                self._transition(ST_RED_LIGHT)
                self.get_logger().info(f'RED_LIGHT: stopping ({self.light_state})')
            self._publish(0.0, 0.0)
            return

        if self.state == ST_RED_LIGHT:
            self.get_logger().info('RED_LIGHT: GREEN, resuming')
            self._transition(ST_LANE)

        # -------------------------------------------------------------------- #
        # PRIORITY 3-6 – STOP SIGN FSM
        # -------------------------------------------------------------------- #
        stop_fsm_result = self._stop_sign_fsm(now, dt)
        if stop_fsm_result:
            return   # stop sign FSM is in control

        # -------------------------------------------------------------------- #
        # PRIORITY 7 – LANE FOLLOWING (with optional overtaking)
        # -------------------------------------------------------------------- #
        self._lane_following_step()

    def _lane_following_step(self):
        """Lane following step with PD steering and speed modulation."""
        if self.state not in (ST_LANE,):
            self._transition(ST_LANE)

        # --- Speed based on LiDAR distance ---
        speed = self.base_speed
        if self.lidar_status == 1:   # slow zone
            speed *= self.slow_speed_factor

        # --- Speed reduction on large steering error ---
        err = self.lane_error
        if abs(err) > self.slow_thr:
            speed *= self.slow_factor

        speed = float(max(self.min_speed, min(self.max_speed, speed)))

        # --- PD steering ---
        w = self._pd_steer(err)

        self._publish(speed, w)

    # ---------------------------------------------------------------------- #
    def _stop_sign_fsm(self, now, dt):
        """Handle the STOP sign state machine.

        Returns True if the FSM is in control (caller should return).
        Returns False if LANE_FOLLOWING should proceed.
        """
        s = self.state

        # ---- State: LANE_FOLLOWING — watch for STOP sign -------------------
        if s == ST_LANE:
            if self.sign_detected == 'STOP':
                self._transition(ST_STOP_DETECTED)
                self.get_logger().info('STOP_DETECTED: approaching')
            return False   # LANE_FOLLOWING proceeds

        # ---- State: STOP_DETECTED — slow down and approach -----------------
        if s == ST_STOP_DETECTED:
            if self.sign_detected != 'STOP' and dt > 0.5:
                # Lost the sign (e.g. drove past it without stopping) → cool
                self._transition(ST_STOP_COOLDOWN)
                return True

            # Slow approach
            speed = self.base_speed * 0.5
            w     = self._pd_steer(self.lane_error)

            # Transition to BRAKE when we are close
            if self.front_dist <= self.stop_brake_dist + 0.20:
                self._transition(ST_STOP_BRAKE)

            self._publish(speed, w)
            return True

        # ---- State: STOP_BRAKE — come to a complete stop -------------------
        if s == ST_STOP_BRAKE:
            self._publish(0.0, 0.0)
            if self.front_dist <= self.stop_brake_dist or dt > 2.5:
                self._transition(ST_STOP_HOLD)
                self.get_logger().info('STOP_HOLD: holding 2 seconds')
            return True

        # ---- State: STOP_HOLD — hold still for at least stop_hold_seconds --
        if s == ST_STOP_HOLD:
            self._publish(0.0, 0.0)
            if dt >= self.stop_hold_s:
                self._stop_sign_count += 1
                self._transition(ST_STOP_COOLDOWN)
                self.get_logger().info(
                    f'STOP_HOLD done (stop #{self._stop_sign_count}), cooldown')
            return True

        # ---- State: STOP_COOLDOWN — ignore sign, drive normally -------------
        if s == ST_STOP_COOLDOWN:
            if dt >= self.stop_cooldown_s:
                self._transition(ST_LANE)
                self.get_logger().info('STOP_COOLDOWN done, resuming LANE')
                return False  # let LANE_FOLLOWING take over this tick

            # Drive normally during cooldown
            speed = self.base_speed
            if self.lidar_status == 1:
                speed *= self.slow_speed_factor
            w     = self._pd_steer(self.lane_error)
            self._publish(speed, w)
            return True

        return False   # not in any STOP state

    # ---------------------------------------------------------------------- #
    def _pd_steer(self, error):
        """PD controller: error (pixels) → angular velocity (rad/s)."""
        deriv = error - self.prev_lane_error
        self.prev_lane_error = error
        w = self.kp * error + self.kd * deriv
        return float(max(-self.max_w, min(self.max_w, w)))

    # ---------------------------------------------------------------------- #
    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x  = float(max(-self.max_speed, min(self.max_speed, v)))
        msg.angular.z = float(max(-self.max_w,     min(self.max_w,     w)))
        self.pub_cmd.publish(msg)

    # ---------------------------------------------------------------------- #
    def _transition(self, new_state):
        old = self.state
        self.state    = new_state
        self.state_t0 = time.time()
        if old != new_state:
            self.get_logger().info(f'FSM: {old} → {new_state}')

    # ---------------------------------------------------------------------- #
    _log_timers = {}

    def _log_rate(self, interval, msg):
        now = time.time()
        if now - self._log_timers.get(msg[:20], 0) >= interval:
            self._log_timers[msg[:20]] = now
            self.get_logger().info(msg)


# ============================================================================ #
def spin(node, stopping):
    while rclpy.ok() and not stopping['now']:
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            if stopping['now'] or not rclpy.ok():
                break
            raise


def main(args=None):
    rclpy.init(args=args)
    stopping = catch_sigterm()
    node = BehaviorNode()
    try:
        spin(node, stopping)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            # Publish zero velocity on exit
            try:
                node._publish(0.0, 0.0)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
