#!/usr/bin/env python3
import rospy
from uuv_sensor_ros_plugins_msgs.msg import AcousticBroadcastMethod3
from std_msgs.msg import String
import threading


class AcousticRouter(object):
    """
    Routes OWTT Method3 broadcast messages from a specific transmitter namespace
    to allowed receivers by republishing on each receiver's subscribed router topic.

    - Input topics (per transmitter): /<tx_ns>/rpt/broadcast_method3
    - Output topics (per receiver): /router_<rx_ns>/rpt/broadcast_method3

    This relies on eca_a9_sensors.xacro setting default_target_ns to router_<namespace>
    so each vehicle subscribes to /router_<self>/rpt/broadcast_method3 only.
    """

    def __init__(self):
        # Load mapping param: dict of tx -> list of rx
        # Example structure:
        # {
        #   'eca_a9_1': ['eca_a9_2','eca_a9_4','eca_a9_5'],
        #   ...
        # }
        self.tx_to_allowed_rx = rospy.get_param('~tx_to_allowed_rx', {})
        if not isinstance(self.tx_to_allowed_rx, dict):
            rospy.logerr('~tx_to_allowed_rx must be a dict')
            self.tx_to_allowed_rx = {}

        # Optionally restrict active TX list; if empty, infer from mapping keys
        self.tx_list = rospy.get_param('~tx_list', [])
        if not self.tx_list:
            self.tx_list = sorted(self.tx_to_allowed_rx.keys())

        # Build publishers per receiver namespace lazily
        self.pub_lock = threading.Lock()
        self.rx_publishers = {}

        # Subscribers per transmitter
        self.subs = []
        for tx in self.tx_list:
            topic = f'/{tx}/rpt/broadcast_method3'
            sub = rospy.Subscriber(topic, AcousticBroadcastMethod3, self._make_cb(tx), queue_size=50)
            self.subs.append(sub)

        rospy.loginfo('AcousticRouter initialized')

    def _get_rx_pub(self, rx_ns):
        with self.pub_lock:
            if rx_ns not in self.rx_publishers:
                topic = f'/router_{rx_ns}/rpt/broadcast_method3'
                self.rx_publishers[rx_ns] = rospy.Publisher(topic, AcousticBroadcastMethod3, queue_size=50)
            return self.rx_publishers[rx_ns]

    def _make_cb(self, tx_ns):
        allowed = self.tx_to_allowed_rx.get(tx_ns, [])

        def cb(msg):
            if not allowed:
                return
            # Single concise log
            rospy.loginfo_throttle(5.0, 'Route TX %s -> [%s]', tx_ns, ', '.join(allowed))
            for rx in allowed:
                pub = self._get_rx_pub(rx)
                pub.publish(msg)
        return cb


def main():
    rospy.init_node('acoustic_router')
    AcousticRouter()
    rospy.spin()


if __name__ == '__main__':
    main()




