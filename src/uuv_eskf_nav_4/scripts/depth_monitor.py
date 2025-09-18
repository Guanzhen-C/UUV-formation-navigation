#!/usr/bin/env python3
import argparse
import threading

import rospy
from sensor_msgs.msg import FluidPressure
from nav_msgs.msg import Odometry


ATM_PRESSURE_KPA = 101.325
KPA_PER_METER = 9.80638


def pressure_to_depth(pressure_kpa: float) -> float:
    if pressure_kpa >= ATM_PRESSURE_KPA:
        return (pressure_kpa - ATM_PRESSURE_KPA) / KPA_PER_METER
    return 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="eca_a9")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--pressure_topic", default=None)
    parser.add_argument("--gt_topic", default=None)
    args = parser.parse_args()

    pressure_topic = args.pressure_topic or f"/{args.robot}/pressure"
    gt_topic = args.gt_topic or f"/{args.robot}/pose_gt"

    rospy.init_node("depth_monitor", anonymous=True)

    lock = threading.Lock()
    latest = {
        "meas": None,
        "t_meas": None,
        "gt": None,
        "t_gt": None,
    }

    def on_pressure(msg: FluidPressure):
        depth = pressure_to_depth(msg.fluid_pressure)
        with lock:
            latest["meas"] = depth
            latest["t_meas"] = msg.header.stamp.to_sec()

    def on_gt(msg: Odometry):
        depth_gt = -msg.pose.pose.position.z
        with lock:
            latest["gt"] = depth_gt
            latest["t_gt"] = msg.header.stamp.to_sec()

    rospy.Subscriber(pressure_topic, FluidPressure, on_pressure, queue_size=10)
    rospy.Subscriber(gt_topic, Odometry, on_gt, queue_size=10)

    rate = rospy.Rate(args.rate)
    end_time = rospy.Time.now().to_sec() + args.duration

    print(f"Subscribing pressure: {pressure_topic}, gt: {gt_topic}")
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() < end_time:
        with lock:
            dm = latest["meas"]
            tg = latest["t_meas"]
            dg = latest["gt"]
            tg2 = latest["t_gt"]
        if dm is not None or dg is not None:
            line = []
            if dm is not None:
                line.append(f"depth_meas={dm:.3f}")
            if dg is not None:
                line.append(f"depth_gt={dg:.3f}")
            if dm is not None and dg is not None:
                line.append(f"diff={dm-dg:+.3f}")
            if tg is not None and tg2 is not None:
                line.append(f"dt={abs(tg-tg2):.3f}s")
            print(" ".join(line))
        rate.sleep()


if __name__ == "__main__":
    main()


