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
        self.cycle_period = float(rospy.get_param('~cycle_period', 0.0))

        # Compute idle phase length to satisfy the requested cycle period
        # Idle happens after the last transmitter in the sequence
        self.idle_duration = 0.0
        if self.cycle_period > 0.0:
            active_time = len(self.sequence) * self.interval
            self.idle_duration = max(0.0, self.cycle_period - active_time)

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
        rospy.loginfo(
            'Scheduling: N=%d, interval=%.3fs, cycle_period=%.3fs, idle=%.3fs',
            len(self.sequence), self.interval, self.cycle_period, self.idle_duration
        )

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
        idle_pending = False  # set True after last transmitter completes its ON interval
        while not rospy.is_shutdown():
            now = time.time()
            if now >= next_switch:
                # If an idle phase is pending, keep everything OFF for idle_duration
                if idle_pending and self.idle_duration > 0.0:
                    for ns in self.sequence:
                        self.set_state(ns, False)
                    rospy.loginfo('Idle phase for %.3f seconds', self.idle_duration)
                    next_switch = now + self.idle_duration
                    idle_pending = False
                else:
                    # Turn off all
                    for ns in self.sequence:
                        self.set_state(ns, False)
                    # Turn on current
                    if len(self.sequence) == 0:
                        # Nothing to schedule; wait a bit to avoid busy loop
                        next_switch = now + 1.0
                    else:
                        current = self.sequence[idx]
                        self.set_state(current, True)
                        rospy.loginfo(
                            'Active transmitter: %s (interval %.3fs)', current, self.interval
                        )
                        idx = (idx + 1) % len(self.sequence)
                        # After the last transmitter completes, schedule an idle phase if requested
                        if idx == 0 and self.idle_duration > 0.0:
                            idle_pending = True
                        next_switch = now + self.interval
            rate.sleep()


def main():
    rospy.init_node('acoustic_scheduler')
    sched = AcousticScheduler()
    sched.run()


if __name__ == '__main__':
    main()




