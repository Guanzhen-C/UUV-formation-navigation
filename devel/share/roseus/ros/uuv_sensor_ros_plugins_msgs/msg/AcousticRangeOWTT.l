;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT)
  (if (not (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS"))
    (make-package "UUV_SENSOR_ROS_PLUGINS_MSGS"))
  (shadow 'AcousticRangeOWTT (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS::ACOUSTICRANGEOWTT")
  (make-package "UUV_SENSOR_ROS_PLUGINS_MSGS::ACOUSTICRANGEOWTT"))

(in-package "ROS")
;;//! \htmlinclude AcousticRangeOWTT.msg.html


(defclass uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT
  :super ros::object
  :slots (_from_ns _to_ns _t_tx _t_rx _range _variance _tx_pos _tx_cross_cov_x_p ))

(defmethod uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT
  (:init
   (&key
    ((:from_ns __from_ns) "")
    ((:to_ns __to_ns) "")
    ((:t_tx __t_tx) (instance ros::time :init))
    ((:t_rx __t_rx) (instance ros::time :init))
    ((:range __range) 0.0)
    ((:variance __variance) 0.0)
    ((:tx_pos __tx_pos) (instance uuv_sensor_ros_plugins_msgs::PositionWithCovarianceStamped :init))
    ((:tx_cross_cov_x_p __tx_cross_cov_x_p) (make-array 45 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _from_ns (string __from_ns))
   (setq _to_ns (string __to_ns))
   (setq _t_tx __t_tx)
   (setq _t_rx __t_rx)
   (setq _range (float __range))
   (setq _variance (float __variance))
   (setq _tx_pos __tx_pos)
   (setq _tx_cross_cov_x_p __tx_cross_cov_x_p)
   self)
  (:from_ns
   (&optional __from_ns)
   (if __from_ns (setq _from_ns __from_ns)) _from_ns)
  (:to_ns
   (&optional __to_ns)
   (if __to_ns (setq _to_ns __to_ns)) _to_ns)
  (:t_tx
   (&optional __t_tx)
   (if __t_tx (setq _t_tx __t_tx)) _t_tx)
  (:t_rx
   (&optional __t_rx)
   (if __t_rx (setq _t_rx __t_rx)) _t_rx)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:variance
   (&optional __variance)
   (if __variance (setq _variance __variance)) _variance)
  (:tx_pos
   (&rest __tx_pos)
   (if (keywordp (car __tx_pos))
       (send* _tx_pos __tx_pos)
     (progn
       (if __tx_pos (setq _tx_pos (car __tx_pos)))
       _tx_pos)))
  (:tx_cross_cov_x_p
   (&optional __tx_cross_cov_x_p)
   (if __tx_cross_cov_x_p (setq _tx_cross_cov_x_p __tx_cross_cov_x_p)) _tx_cross_cov_x_p)
  (:serialization-length
   ()
   (+
    ;; string _from_ns
    4 (length _from_ns)
    ;; string _to_ns
    4 (length _to_ns)
    ;; time _t_tx
    8
    ;; time _t_rx
    8
    ;; float64 _range
    8
    ;; float64 _variance
    8
    ;; uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped _tx_pos
    (send _tx_pos :serialization-length)
    ;; float64[45] _tx_cross_cov_x_p
    (* 8    45)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _from_ns
       (write-long (length _from_ns) s) (princ _from_ns s)
     ;; string _to_ns
       (write-long (length _to_ns) s) (princ _to_ns s)
     ;; time _t_tx
       (write-long (send _t_tx :sec) s) (write-long (send _t_tx :nsec) s)
     ;; time _t_rx
       (write-long (send _t_rx :sec) s) (write-long (send _t_rx :nsec) s)
     ;; float64 _range
       (sys::poke _range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _variance
       (sys::poke _variance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped _tx_pos
       (send _tx_pos :serialize s)
     ;; float64[45] _tx_cross_cov_x_p
     (dotimes (i 45)
       (sys::poke (elt _tx_cross_cov_x_p i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _from_ns
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _from_ns (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _to_ns
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _to_ns (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _t_tx
     (send _t_tx :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t_tx :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _t_rx
     (send _t_rx :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t_rx :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _range
     (setq _range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _variance
     (setq _variance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped _tx_pos
     (send _tx_pos :deserialize buf ptr-) (incf ptr- (send _tx_pos :serialization-length))
   ;; float64[45] _tx_cross_cov_x_p
   (dotimes (i (length _tx_cross_cov_x_p))
     (setf (elt _tx_cross_cov_x_p i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT :md5sum-) "8215537df9cb58042c7c2090621e64e6")
(setf (get uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT :datatype-) "uuv_sensor_ros_plugins_msgs/AcousticRangeOWTT")
(setf (get uuv_sensor_ros_plugins_msgs::AcousticRangeOWTT :definition-)
      "string from_ns
string to_ns
# Transmit time at sender and receive time at receiver (ROS sim time)
time t_tx
time t_rx

# One-way measured range (meters) and its variance
float64 range
float64 variance

# Transmitter's position estimate and covariance at t_tx
uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped tx_pos

# Cross-covariance between sender 15-dim error state and its position (15x3, row-major)
float64[45] tx_cross_cov_x_p



================================================================================
MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This expresses an estimated position with a reference coordinate frame and
# timestamp

Header header
PositionWithCovariance pos

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovariance
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This represents a position in free space with uncertainty.

geometry_msgs/Point pos

# Row-major representation of the 3x3 covariance matrix
float64[9] covariance

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :uuv_sensor_ros_plugins_msgs/AcousticRangeOWTT "8215537df9cb58042c7c2090621e64e6")


