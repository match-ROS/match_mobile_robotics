#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from threading import Lock

class JointStateAggregator:
    def __init__(self):
        self.lock = Lock()
        self.state = {}  # name -> (pos, vel, eff)
        self.have_vel = False
        self.have_eff = False

        # Params
        self.topic_mir  = rospy.get_param("~topic_mir",  "")
        self.topic_ur_l   = rospy.get_param("~topic_ur_l",   "")
        self.topic_ur_r   = rospy.get_param("~topic_ur_r",   "")
        self.topic_lift_l = rospy.get_param("~topic_lift_l", "")
        self.topic_lift_r = rospy.get_param("~topic_lift_r", "")
        self.out_topic  = rospy.get_param("~out_topic",  "joint_states")
        self.frame_id   = rospy.get_param("~frame_id",   "")

        # Publisher
        self.pub = rospy.Publisher(self.out_topic, JointState, queue_size=10)

        # Subscribers (only if topic is set)
        if self.topic_mir:
            rospy.Subscriber(self.topic_mir, JointState, self._cb_cache, queue_size=5, tcp_nodelay=True)
        if self.topic_lift_l:
            rospy.Subscriber(self.topic_lift_l, JointState, self._cb_cache, queue_size=5, tcp_nodelay=True)
        if self.topic_lift_r:
            rospy.Subscriber(self.topic_lift_r, JointState, self._cb_cache, queue_size=5, tcp_nodelay=True)
        if self.topic_ur_l:
            rospy.Subscriber(self.topic_ur_l, JointState, self._cb_cache, queue_size=5, tcp_nodelay=True)
            self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 500.0)
        if self.topic_ur_r:
            self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 500.0)
            rospy.Subscriber(self.topic_ur_r, JointState, self._cb_cache, queue_size=5, tcp_nodelay=True)
        if not self.topic_ur_l and  not self.topic_ur_r:
            self.publish_rate_hz = 100.0
            rospy.logwarn("~topic_ur is empty -> publishing periodically at %.1f Hz", self.publish_rate_hz)
        
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self._timer_publish)
        rospy.loginfo("JointStateAggregator out=%s", self.out_topic)
        rospy.loginfo("  mir:  %s", self.topic_mir)
        rospy.loginfo("  ur:   %s", self.topic_ur_l)
        rospy.loginfo("  lift: %s", self.topic_lift_l)

    def _cb_cache(self, msg: JointState):
        # Cache only
        with self.lock:
            self._merge(msg)

    def _merge(self, msg: JointState):
        n = len(msg.name)
        pos = msg.position if len(msg.position) == n else None
        vel = msg.velocity if len(msg.velocity) == n else None
        eff = msg.effort   if len(msg.effort)   == n else None

        if vel is not None:
            self.have_vel = True
        if eff is not None:
            self.have_eff = True

        for i, name in enumerate(msg.name):
            p = pos[i] if pos is not None else None
            v = vel[i] if vel is not None else None
            e = eff[i] if eff is not None else None

            old = self.state.get(name, (None, None, None))
            # Update only fields we have in this message; keep previous otherwise
            self.state[name] = (
                p if p is not None else old[0],
                v if v is not None else old[1],
                e if e is not None else old[2],
            )

    def _build_output(self, stamp):
        out = JointState()
        out.header.stamp = stamp if stamp and stamp != rospy.Time(0) else rospy.Time.now()
        if self.frame_id:
            out.header.frame_id = self.frame_id

        names = sorted(self.state.keys())
        out.name = names

        # Always fill position; fill vel/eff only if we ever saw consistent arrays
        out.position = [self.state[n][0] if self.state[n][0] is not None else 0.0 for n in names]
        if self.have_vel:
            out.velocity = [self.state[n][1] if self.state[n][1] is not None else 0.0 for n in names]
        if self.have_eff:
            out.effort   = [self.state[n][2] if self.state[n][2] is not None else 0.0 for n in names]
        return out

    def _timer_publish(self, _event):
        with self.lock:
            out = self._build_output(stamp=rospy.Time.now())
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("joint_state_aggregator")
    JointStateAggregator()
    rospy.spin()
