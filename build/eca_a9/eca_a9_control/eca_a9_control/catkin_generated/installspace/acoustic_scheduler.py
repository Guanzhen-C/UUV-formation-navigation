#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Bool


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
        self.enable_rx_tx = bool(rospy.get_param('~enable_rx_tx', False))

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

        # Prepare tx_enable publishers per vehicle
        self.tx_enable_pubs = {}
        for ns in self.sequence:
            topic = f'/{ns}/{self.rpt_topic}/tx_enable'
            self.tx_enable_pubs[ns] = rospy.Publisher(topic, Bool, queue_size=1, latch=True)

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
            rospy.logdebug(f'Set {ns} RPT {"ON" if on else "OFF"}: {resp.message}')
            return True
        except Exception as e:
            rospy.logwarn(f'Failed to set {ns} state {on}: {e}')
            return False

    def set_tx_enable(self, ns, enable):
        try:
            pub = self.tx_enable_pubs.get(ns)
            if pub is not None:
                pub.publish(Bool(data=enable))
            return True
        except Exception as e:
            rospy.logwarn(f'Failed to set {ns} tx_enable {enable}: {e}')
            return False

    def run(self):
        # Ensure all off first
        for ns in self.sequence:
            self.set_state(ns, False)
            self.set_tx_enable(ns, False)
        idx = 0
        rate = rospy.Rate(1.0)
        # Use ROS time (sim time if use_sim_time=true)
        # Wait for sim time to start if /clock is not yet published
        if rospy.Time.now().to_sec() == 0.0:
            rospy.loginfo('Waiting for ROS time to start...')
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0.0:
                rate.sleep()
        next_switch = rospy.get_time()
        idle_pending = False  # set True after last transmitter completes its ON interval
        while not rospy.is_shutdown():
            now = rospy.get_time()
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
                        self.set_tx_enable(ns, False)
                    # Turn on current
                    if len(self.sequence) == 0:
                        # Nothing to schedule; wait a bit to avoid busy loop
                        next_switch = now + 1.0
                    else:
                        current = self.sequence[idx]
                        self.set_state(current, True)
                        self.set_tx_enable(current, True)  # only current TX broadcasts
                        # Log send-time routing and enable receivers as well
                        try:
                            mapping = rospy.get_param('/acoustic_router/tx_to_allowed_rx', {})
                            allowed = mapping.get(current, []) if isinstance(mapping, dict) else []
                            rospy.loginfo('TX %s | RX [%s] t=%.3f', current, ', '.join(allowed), rospy.get_time())
                            # Optionally turn on receivers (processing only, TX disabled)
                            if self.enable_rx_tx:
                                # Enable both RX and TX on receivers (legacy behavior)
                                for rx in allowed:
                                    if rx != current and rx in self.proxies:
                                        self.set_state(rx, True)
                                        self.set_tx_enable(rx, True)
                            else:
                                # Receivers ON to compute ranges, but TX disabled to avoid flood
                                for rx in allowed:
                                    if rx != current and rx in self.proxies:
                                        self.set_state(rx, True)
                                        self.set_tx_enable(rx, False)
                        except Exception as e:
                            rospy.logwarn('Failed to apply routing for %s: %s', current, e)
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




