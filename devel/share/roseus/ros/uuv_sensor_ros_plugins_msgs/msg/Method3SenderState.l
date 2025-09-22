;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_ros_plugins_msgs::Method3SenderState)
  (if (not (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS"))
    (make-package "UUV_SENSOR_ROS_PLUGINS_MSGS"))
  (shadow 'Method3SenderState (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS::METHOD3SENDERSTATE")
  (make-package "UUV_SENSOR_ROS_PLUGINS_MSGS::METHOD3SENDERSTATE"))

(in-package "ROS")
;;//! \htmlinclude Method3SenderState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_sensor_ros_plugins_msgs::Method3SenderState
  :super ros::object
  :slots (_header _ns _pos _cross_cov_x_p ))

(defmethod uuv_sensor_ros_plugins_msgs::Method3SenderState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ns __ns) "")
    ((:pos __pos) (instance uuv_sensor_ros_plugins_msgs::PositionWithCovarianceStamped :init))
    ((:cross_cov_x_p __cross_cov_x_p) (make-array 45 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _ns (string __ns))
   (setq _pos __pos)
   (setq _cross_cov_x_p __cross_cov_x_p)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ns
   (&optional __ns)
   (if __ns (setq _ns __ns)) _ns)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:cross_cov_x_p
   (&optional __cross_cov_x_p)
   (if __cross_cov_x_p (setq _cross_cov_x_p __cross_cov_x_p)) _cross_cov_x_p)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _ns
    4 (length _ns)
    ;; uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped _pos
    (send _pos :serialization-length)
    ;; float64[45] _cross_cov_x_p
    (* 8    45)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _ns
       (write-long (length _ns) s) (princ _ns s)
     ;; uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped _pos
       (send _pos :serialize s)
     ;; float64[45] _cross_cov_x_p
     (dotimes (i 45)
       (sys::poke (elt _cross_cov_x_p i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _ns
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ns (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped _pos
     (send _pos :deserialize buf ptr-) (incf ptr- (send _pos :serialization-length))
   ;; float64[45] _cross_cov_x_p
   (dotimes (i (length _cross_cov_x_p))
     (setf (elt _cross_cov_x_p i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get uuv_sensor_ros_plugins_msgs::Method3SenderState :md5sum-) "76191afa343ce0995d28537435cdb145")
(setf (get uuv_sensor_ros_plugins_msgs::Method3SenderState :datatype-) "uuv_sensor_ros_plugins_msgs/Method3SenderState")
(setf (get uuv_sensor_ros_plugins_msgs::Method3SenderState :definition-)
      "Header header
string ns
uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped pos
float64[45] cross_cov_x_p


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



(provide :uuv_sensor_ros_plugins_msgs/Method3SenderState "76191afa343ce0995d28537435cdb145")


