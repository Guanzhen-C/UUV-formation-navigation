; Auto-generated. Do not edit!


(cl:in-package uuv_sensor_ros_plugins_msgs-msg)


;//! \htmlinclude AcousticRangeOWTT.msg.html

(cl:defclass <AcousticRangeOWTT> (roslisp-msg-protocol:ros-message)
  ((from_ns
    :reader from_ns
    :initarg :from_ns
    :type cl:string
    :initform "")
   (to_ns
    :reader to_ns
    :initarg :to_ns
    :type cl:string
    :initform "")
   (t_tx
    :reader t_tx
    :initarg :t_tx
    :type cl:real
    :initform 0)
   (t_rx
    :reader t_rx
    :initarg :t_rx
    :type cl:real
    :initform 0)
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0)
   (variance
    :reader variance
    :initarg :variance
    :type cl:float
    :initform 0.0)
   (tx_pos
    :reader tx_pos
    :initarg :tx_pos
    :type uuv_sensor_ros_plugins_msgs-msg:PositionWithCovarianceStamped
    :initform (cl:make-instance 'uuv_sensor_ros_plugins_msgs-msg:PositionWithCovarianceStamped))
   (tx_cross_cov_x_p
    :reader tx_cross_cov_x_p
    :initarg :tx_cross_cov_x_p
    :type (cl:vector cl:float)
   :initform (cl:make-array 45 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass AcousticRangeOWTT (<AcousticRangeOWTT>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AcousticRangeOWTT>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AcousticRangeOWTT)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_ros_plugins_msgs-msg:<AcousticRangeOWTT> is deprecated: use uuv_sensor_ros_plugins_msgs-msg:AcousticRangeOWTT instead.")))

(cl:ensure-generic-function 'from_ns-val :lambda-list '(m))
(cl:defmethod from_ns-val ((m <AcousticRangeOWTT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:from_ns-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:from_ns instead.")
  (from_ns m))

(cl:ensure-generic-function 'to_ns-val :lambda-list '(m))
(cl:defmethod to_ns-val ((m <AcousticRangeOWTT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:to_ns-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:to_ns instead.")
  (to_ns m))

(cl:ensure-generic-function 't_tx-val :lambda-list '(m))
(cl:defmethod t_tx-val ((m <AcousticRangeOWTT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:t_tx-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:t_tx instead.")
  (t_tx m))

(cl:ensure-generic-function 't_rx-val :lambda-list '(m))
(cl:defmethod t_rx-val ((m <AcousticRangeOWTT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:t_rx-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:t_rx instead.")
  (t_rx m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <AcousticRangeOWTT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:range-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'variance-val :lambda-list '(m))
(cl:defmethod variance-val ((m <AcousticRangeOWTT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:variance-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:variance instead.")
  (variance m))

(cl:ensure-generic-function 'tx_pos-val :lambda-list '(m))
(cl:defmethod tx_pos-val ((m <AcousticRangeOWTT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:tx_pos-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:tx_pos instead.")
  (tx_pos m))

(cl:ensure-generic-function 'tx_cross_cov_x_p-val :lambda-list '(m))
(cl:defmethod tx_cross_cov_x_p-val ((m <AcousticRangeOWTT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:tx_cross_cov_x_p-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:tx_cross_cov_x_p instead.")
  (tx_cross_cov_x_p m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AcousticRangeOWTT>) ostream)
  "Serializes a message object of type '<AcousticRangeOWTT>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'from_ns))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'from_ns))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'to_ns))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'to_ns))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 't_tx)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 't_tx) (cl:floor (cl:slot-value msg 't_tx)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 't_rx)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 't_rx) (cl:floor (cl:slot-value msg 't_rx)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'variance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tx_pos) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'tx_cross_cov_x_p))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AcousticRangeOWTT>) istream)
  "Deserializes a message object of type '<AcousticRangeOWTT>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'from_ns) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'from_ns) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'to_ns) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'to_ns) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 't_tx) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 't_rx) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'variance) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tx_pos) istream)
  (cl:setf (cl:slot-value msg 'tx_cross_cov_x_p) (cl:make-array 45))
  (cl:let ((vals (cl:slot-value msg 'tx_cross_cov_x_p)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AcousticRangeOWTT>)))
  "Returns string type for a message object of type '<AcousticRangeOWTT>"
  "uuv_sensor_ros_plugins_msgs/AcousticRangeOWTT")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AcousticRangeOWTT)))
  "Returns string type for a message object of type 'AcousticRangeOWTT"
  "uuv_sensor_ros_plugins_msgs/AcousticRangeOWTT")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AcousticRangeOWTT>)))
  "Returns md5sum for a message object of type '<AcousticRangeOWTT>"
  "8215537df9cb58042c7c2090621e64e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AcousticRangeOWTT)))
  "Returns md5sum for a message object of type 'AcousticRangeOWTT"
  "8215537df9cb58042c7c2090621e64e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AcousticRangeOWTT>)))
  "Returns full string definition for message of type '<AcousticRangeOWTT>"
  (cl:format cl:nil "string from_ns~%string to_ns~%# Transmit time at sender and receive time at receiver (ROS sim time)~%time t_tx~%time t_rx~%~%# One-way measured range (meters) and its variance~%float64 range~%float64 variance~%~%# Transmitter's position estimate and covariance at t_tx~%uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped tx_pos~%~%# Cross-covariance between sender 15-dim error state and its position (15x3, row-major)~%float64[45] tx_cross_cov_x_p~%~%~%~%================================================================================~%MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This expresses an estimated position with a reference coordinate frame and~%# timestamp~%~%Header header~%PositionWithCovariance pos~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovariance~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This represents a position in free space with uncertainty.~%~%geometry_msgs/Point pos~%~%# Row-major representation of the 3x3 covariance matrix~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AcousticRangeOWTT)))
  "Returns full string definition for message of type 'AcousticRangeOWTT"
  (cl:format cl:nil "string from_ns~%string to_ns~%# Transmit time at sender and receive time at receiver (ROS sim time)~%time t_tx~%time t_rx~%~%# One-way measured range (meters) and its variance~%float64 range~%float64 variance~%~%# Transmitter's position estimate and covariance at t_tx~%uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped tx_pos~%~%# Cross-covariance between sender 15-dim error state and its position (15x3, row-major)~%float64[45] tx_cross_cov_x_p~%~%~%~%================================================================================~%MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovarianceStamped~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This expresses an estimated position with a reference coordinate frame and~%# timestamp~%~%Header header~%PositionWithCovariance pos~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: uuv_sensor_ros_plugins_msgs/PositionWithCovariance~%# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%# This represents a position in free space with uncertainty.~%~%geometry_msgs/Point pos~%~%# Row-major representation of the 3x3 covariance matrix~%float64[9] covariance~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AcousticRangeOWTT>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'from_ns))
     4 (cl:length (cl:slot-value msg 'to_ns))
     8
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tx_pos))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tx_cross_cov_x_p) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AcousticRangeOWTT>))
  "Converts a ROS message object to a list"
  (cl:list 'AcousticRangeOWTT
    (cl:cons ':from_ns (from_ns msg))
    (cl:cons ':to_ns (to_ns msg))
    (cl:cons ':t_tx (t_tx msg))
    (cl:cons ':t_rx (t_rx msg))
    (cl:cons ':range (range msg))
    (cl:cons ':variance (variance msg))
    (cl:cons ':tx_pos (tx_pos msg))
    (cl:cons ':tx_cross_cov_x_p (tx_cross_cov_x_p msg))
))
