#!/usr/bin/env python3
import rospy
import time
from std_srvs.srv import SetBool


class AcousticScheduler(object):
    """
    Powers on the RPT sensor (thus enabling broadcast) for one vehicle at a time
    in the sequence eca_a9_1..eca_a9_9, switching every interval seconds.

    It calls the ChangeSensorState service exposed by the RPT plugin at
    /<ns>/rpt/change_state with std_srvs/SetBool? No: plugin uses custom srv.

    The plugin actually advertises uuv_sensor_ros_plugins_msgs/ChangeSensorState
    at /<ns>/rpt/change_state. We'll import the service type dynamically.
    """

    def __init__(self):
        self.sequence = rospy.get_param('~sequence', [
            'eca_a9_1','eca_a9_2','eca_a9_3','eca_a9_4','eca_a9_5',
            'eca_a9_6','eca_a9_7','eca_a9_8','eca_a9_9'
        ])
        self.interval = float(rospy.get_param('~interval', 20.0))
        self.rpt_topic = rospy.get_param('~rpt_topic', 'rpt')

        # Import service lazily
        from uuv_sensor_ros_plugins_msgs.srv import ChangeSensorState
        self.srv_type = ChangeSensorState

        # Prepare service proxies
        self.proxies = {}
        for ns in self.sequence:
            srv_name = f'/{ns}/{self.rpt_topic}/change_state'
            self.proxies[ns] = rospy.ServiceProxy(srv_name, self.srv_type)

        # Wait for all services
        for ns in self.sequence:
            srv_name = f'/{ns}/{self.rpt_topic}/change_state'
            rospy.loginfo(f'Waiting for {srv_name} ...')
            try:
                self.proxies[ns].wait_for_service(timeout=30.0)
            except rospy.ROSException:
                rospy.logwarn(f'Service {srv_name} not available yet; will keep trying during loop')

        rospy.loginfo('AcousticScheduler initialized')

    def set_state(self, ns, on):
        try:
            resp = self.proxies[ns](on)
            rospy.loginfo(f'Set {ns} RPT {"ON" if on else "OFF"}: {resp.message}')
            return True
        except Exception as e:
            rospy.logwarn(f'Failed to set {ns} state {on}: {e}')
            return False

    def run(self):
        # Ensure all off first
        for ns in self.sequence:
            self.set_state(ns, False)
        idx = 0
        rate = rospy.Rate(1.0)
        # Use wall time to avoid sim time pausing effects
        next_switch = time.time()
        while not rospy.is_shutdown():
            now = time.time()
            if now >= next_switch:
                # Turn off all
                for ns in self.sequence:
                    self.set_state(ns, False)
                # Turn on current
                current = self.sequence[idx]
                self.set_state(current, True)
                rospy.loginfo(f'Active transmitter: {current} (interval {self.interval}s)')
                idx = (idx + 1) % len(self.sequence)
                next_switch = now + self.interval
            rate.sleep()


def main():
    rospy.init_node('acoustic_scheduler')
    sched = AcousticScheduler()
    sched.run()


if __name__ == '__main__':
    main()




