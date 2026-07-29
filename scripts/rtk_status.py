#!/usr/bin/env python3
"""Report RTK health from ROS, without touching the serial port.

The command-line tools in tools/ open the receiver directly, so they CANNOT run
while ublox_gps has the port. Once the driver is up, use this node instead.

Subscribes to the driver's NavPVT and reports the two things that actually
matter in the field: whether corrections are being applied, and how good the
solution is. Logs a warning when RTK drops so a degraded run is visible in the
rosbag rather than discovered afterwards.

    rosrun ublox_utils rtk_status.py __ns:=ublox_rover
    roslaunch ublox_utils rover.launch rtk_status:=true

Publishes:
    ~rtk_state  (std_msgs/String)       none | FLOAT | FIXED
    ~rtk_ok     (std_msgs/Bool)         true when carrier solution is FLOAT/FIXED
"""

import rospy
from std_msgs.msg import Bool, String
from ublox_msgs.msg import NavPVT

CARR = {0: "none", 1: "FLOAT", 2: "FIXED"}
FIX = {0: "no fix", 1: "dead reckoning", 2: "2D", 3: "3D",
       4: "GNSS+DR", 5: "time only"}


class RtkStatus(object):
    def __init__(self):
        self.report_period = rospy.get_param("~report_period", 5.0)
        # Below this, a FIXED claim is not trustworthy.
        self.min_sats = rospy.get_param("~min_satellites", 6)

        # Relative, not private (~): the node runs inside the driver's namespace,
        # so these land next to fix/navpvt as /<ns>/rtk_state rather than being
        # buried at /<ns>/rtk_status/rtk_state.
        self.pub_state = rospy.Publisher("rtk_state", String, queue_size=1, latch=True)
        self.pub_ok = rospy.Publisher("rtk_ok", Bool, queue_size=1, latch=True)

        self.last_state = None
        self.last_report = rospy.Time(0)
        self.last_msg_time = None

        rospy.Subscriber("navpvt", NavPVT, self.on_navpvt, queue_size=10)
        rospy.Timer(rospy.Duration(2.0), self.check_alive)
        rospy.loginfo("rtk_status: waiting for navpvt ...")

    def on_navpvt(self, msg):
        self.last_msg_time = rospy.Time.now()

        carr = (msg.flags >> 6) & 0x03
        state = CARR.get(carr, "?")
        diff = bool(msg.flags & 0x02)
        h_acc = msg.hAcc / 1000.0

        if state != self.last_state:
            if self.last_state is not None:
                if carr == 0:
                    rospy.logwarn("rtk_status: RTK LOST (%s -> %s), hAcc %.3f m"
                                  % (self.last_state, state, h_acc))
                else:
                    rospy.loginfo("rtk_status: RTK %s -> %s, hAcc %.3f m"
                                  % (self.last_state, state, h_acc))
            self.last_state = state
            self.pub_state.publish(String(data=state))
            self.pub_ok.publish(Bool(data=carr in (1, 2)))

        now = rospy.Time.now()
        if (now - self.last_report).to_sec() >= self.report_period:
            self.last_report = now
            warn = ""
            if carr == 0:
                warn = ("  <-- no carrier solution; corrections not applied"
                        if not diff else
                        "  <-- DGNSS only, no carrier solution")
            elif msg.numSV < self.min_sats:
                warn = "  <-- only %d satellites, solution is fragile" % msg.numSV
            rospy.loginfo("rtk_status: %s RTK=%s sv=%d hAcc=%.3f m%s"
                          % (FIX.get(msg.fixType, "?"), state, msg.numSV, h_acc, warn))

    def check_alive(self, _):
        if self.last_msg_time is None:
            return
        gap = (rospy.Time.now() - self.last_msg_time).to_sec()
        if gap > 3.0:
            rospy.logwarn_throttle(
                10.0, "rtk_status: no navpvt for %.1f s -- is the receiver "
                      "connected and is ublox_gps running?" % gap)


if __name__ == "__main__":
    rospy.init_node("rtk_status")
    RtkStatus()
    rospy.spin()
