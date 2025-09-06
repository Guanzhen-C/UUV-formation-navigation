; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-msg)


;//! \htmlinclude ParticleConcentration.msg.html

(cl:defclass <ParticleConcentration> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (geo_point
    :reader geo_point
    :initarg :geo_point
    :type geographic_msgs-msg:GeoPoint
    :initform (cl:make-instance 'geographic_msgs-msg:GeoPoint))
   (concentration
    :reader concentration
    :initarg :concentration
    :type cl:float
    :initform 0.0))
)

(cl:defclass ParticleConcentration (<ParticleConcentration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParticleConcentration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParticleConcentration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-msg:<ParticleConcentration> is deprecated: use uuv_plume_msgs-msg:ParticleConcentration instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-msg:header-val is deprecated.  Use uuv_plume_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-msg:position-val is deprecated.  Use uuv_plume_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'geo_point-val :lambda-list '(m))
(cl:defmethod geo_point-val ((m <ParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-msg:geo_point-val is deprecated.  Use uuv_plume_msgs-msg:geo_point instead.")
  (geo_point m))

(cl:ensure-generic-function 'concentration-val :lambda-list '(m))
(cl:defmethod concentration-val ((m <ParticleConcentration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-msg:concentration-val is deprecated.  Use uuv_plume_msgs-msg:concentration instead.")
  (concentration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParticleConcentration>) ostream)
  "Serializes a message object of type '<ParticleConcentration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'geo_point) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'concentration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParticleConcentration>) istream)
  "Deserializes a message object of type '<ParticleConcentration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'geo_point) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'concentration) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParticleConcentration>)))
  "Returns string type for a message object of type '<ParticleConcentration>"
  "uuv_plume_msgs/ParticleConcentration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParticleConcentration)))
  "Returns string type for a message object of type 'ParticleConcentration"
  "uuv_plume_msgs/ParticleConcentration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParticleConcentration>)))
  "Returns md5sum for a message object of type '<ParticleConcentration>"
  "018ff018a5cc3c75eb3684e0da416577")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParticleConcentration)))
  "Returns md5sum for a message object of type 'ParticleConcentration"
  "018ff018a5cc3c75eb3684e0da416577")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParticleConcentration>)))
  "Returns full string definition for message of type '<ParticleConcentration>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%Header header~%# Sensor link position wrt the reference frame~%geometry_msgs/Vector3 position~%# WGS84 coordinates (the altitude component corresponds to -1 * depth)~%geographic_msgs/GeoPoint geo_point~%# Particle concentration~%float64 concentration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geographic_msgs/GeoPoint~%# Geographic point, using the WGS 84 reference ellipsoid.~%~%# Latitude [degrees]. Positive is north of equator; negative is south~%# (-90 <= latitude <= +90).~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is~%# west (-180 <= longitude <= +180). At the poles, latitude is -90 or~%# +90, and longitude is irrelevant, but must be in range.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParticleConcentration)))
  "Returns full string definition for message of type 'ParticleConcentration"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%Header header~%# Sensor link position wrt the reference frame~%geometry_msgs/Vector3 position~%# WGS84 coordinates (the altitude component corresponds to -1 * depth)~%geographic_msgs/GeoPoint geo_point~%# Particle concentration~%float64 concentration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geographic_msgs/GeoPoint~%# Geographic point, using the WGS 84 reference ellipsoid.~%~%# Latitude [degrees]. Positive is north of equator; negative is south~%# (-90 <= latitude <= +90).~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is~%# west (-180 <= longitude <= +180). At the poles, latitude is -90 or~%# +90, and longitude is irrelevant, but must be in range.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).~%float64 altitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParticleConcentration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'geo_point))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParticleConcentration>))
  "Converts a ROS message object to a list"
  (cl:list 'ParticleConcentration
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':geo_point (geo_point msg))
    (cl:cons ':concentration (concentration msg))
))
