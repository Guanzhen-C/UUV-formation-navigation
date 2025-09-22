; Auto-generated. Do not edit!


(cl:in-package uuv_sensor_ros_plugins_msgs-msg)


;//! \htmlinclude Method3SenderState.msg.html

(cl:defclass <Method3SenderState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ns
    :reader ns
    :initarg :ns
    :type cl:string
    :initform "")
   (pos
    :reader pos
    :initarg :pos
    :type uuv_sensor_ros_plugins_msgs-msg:PositionWithCovarianceStamped
    :initform (cl:make-instance 'uuv_sensor_ros_plugins_msgs-msg:PositionWithCovarianceStamped))
   (cross_cov_x_p
    :reader cross_cov_x_p
    :initarg :cross_cov_x_p
    :type (cl:vector cl:float)
   :initform (cl:make-array 45 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Method3SenderState (<Method3SenderState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Method3SenderState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Method3SenderState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_ros_plugins_msgs-msg:<Method3SenderState> is deprecated: use uuv_sensor_ros_plugins_msgs-msg:Method3SenderState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Method3SenderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:header-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ns-val :lambda-list '(m))
(cl:defmethod ns-val ((m <Method3SenderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:ns-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:ns instead.")
  (ns m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <Method3SenderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:pos-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'cross_cov_x_p-val :lambda-list '(m))
(cl:defmethod cross_cov_x_p-val ((m <Method3SenderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:cross_cov_x_p-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:cross_cov_x_p instead.")
  (cross_cov_x_p m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Method3SenderState>) ostream)
  "Serializes a message object of type '<Method3SenderState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ns))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ns))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'cross_cov_x_p))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Method3SenderState>) istream)
  "Deserializes a message object of type '<Method3SenderState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ns) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ns) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
  (cl:setf (cl:slot-value msg 'cross_cov_x_p) (cl:make-array 45))
  (cl:let ((vals (cl:slot-value msg 'cross_cov_x_p)))
    (cl:dotimes (i 45)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Method3SenderState>)))
  "Returns string type for a message object of type '<Method3SenderState>"
  "uuv_sensor_ros_plugins_msgs/Method3SenderState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Method3SenderState)))
  "Returns string type for a message object of type 'Method3SenderState"
  "uuv_sensor_ros_plugins_msgs/Method3SenderState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Method3SenderState>)))
  "Returns md5sum for a message object of type '<Method3SenderState>"
  "76191afa343ce0995d28537435cdb145")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Method3SenderState)))
  "Returns md5sum for a message object of type 'Method3SenderState"
  "76191afa343ce0995d28537435cdb145")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Method3SenderState>)))
  "Returns full string definition for message of type '<Method3SenderState>"
  (cl:format cl:nil "Header header~%string ns~%uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped pos~%float64[45] cross_cov_x_p~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This expresses an estimated position with a reference coordinate frame and~%# timestamp~%~%Header header~%PositionWithCovariance pos~%~%================================================================================~%MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovariance~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This represents a position in free space with uncertainty.~%~%geometry_msgs/Point pos~%~%# Row-major representation of the 3x3 covariance matrix~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Method3SenderState)))
  "Returns full string definition for message of type 'Method3SenderState"
  (cl:format cl:nil "Header header~%string ns~%uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped pos~%float64[45] cross_cov_x_p~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This expresses an estimated position with a reference coordinate frame and~%# timestamp~%~%Header header~%PositionWithCovariance pos~%~%================================================================================~%MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovariance~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This represents a position in free space with uncertainty.~%~%geometry_msgs/Point pos~%~%# Row-major representation of the 3x3 covariance matrix~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Method3SenderState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'ns))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'cross_cov_x_p) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Method3SenderState>))
  "Converts a ROS message object to a list"
  (cl:list 'Method3SenderState
    (cl:cons ':header (header msg))
    (cl:cons ':ns (ns msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':cross_cov_x_p (cross_cov_x_p msg))
))
