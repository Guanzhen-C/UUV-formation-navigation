; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-srv)


;//! \htmlinclude SetPlumeConfiguration-request.msg.html

(cl:defclass <SetPlumeConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((n_points
    :reader n_points
    :initarg :n_points
    :type cl:integer
    :initform 0)
   (max_particles_per_iter
    :reader max_particles_per_iter
    :initarg :max_particles_per_iter
    :type cl:integer
    :initform 0))
)

(cl:defclass SetPlumeConfiguration-request (<SetPlumeConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPlumeConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPlumeConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<SetPlumeConfiguration-request> is deprecated: use uuv_plume_msgs-srv:SetPlumeConfiguration-request instead.")))

(cl:ensure-generic-function 'n_points-val :lambda-list '(m))
(cl:defmethod n_points-val ((m <SetPlumeConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:n_points-val is deprecated.  Use uuv_plume_msgs-srv:n_points instead.")
  (n_points m))

(cl:ensure-generic-function 'max_particles_per_iter-val :lambda-list '(m))
(cl:defmethod max_particles_per_iter-val ((m <SetPlumeConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:max_particles_per_iter-val is deprecated.  Use uuv_plume_msgs-srv:max_particles_per_iter instead.")
  (max_particles_per_iter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPlumeConfiguration-request>) ostream)
  "Serializes a message object of type '<SetPlumeConfiguration-request>"
  (cl:let* ((signed (cl:slot-value msg 'n_points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'max_particles_per_iter)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPlumeConfiguration-request>) istream)
  "Deserializes a message object of type '<SetPlumeConfiguration-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_points) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_particles_per_iter) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPlumeConfiguration-request>)))
  "Returns string type for a service object of type '<SetPlumeConfiguration-request>"
  "uuv_plume_msgs/SetPlumeConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPlumeConfiguration-request)))
  "Returns string type for a service object of type 'SetPlumeConfiguration-request"
  "uuv_plume_msgs/SetPlumeConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPlumeConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetPlumeConfiguration-request>"
  "55045be54c5c1ddb03ae4cfc1750c6aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPlumeConfiguration-request)))
  "Returns md5sum for a message object of type 'SetPlumeConfiguration-request"
  "55045be54c5c1ddb03ae4cfc1750c6aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPlumeConfiguration-request>)))
  "Returns full string definition for message of type '<SetPlumeConfiguration-request>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%int32 n_points~%int32 max_particles_per_iter~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPlumeConfiguration-request)))
  "Returns full string definition for message of type 'SetPlumeConfiguration-request"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%int32 n_points~%int32 max_particles_per_iter~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPlumeConfiguration-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPlumeConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPlumeConfiguration-request
    (cl:cons ':n_points (n_points msg))
    (cl:cons ':max_particles_per_iter (max_particles_per_iter msg))
))
;//! \htmlinclude SetPlumeConfiguration-response.msg.html

(cl:defclass <SetPlumeConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetPlumeConfiguration-response (<SetPlumeConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPlumeConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPlumeConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<SetPlumeConfiguration-response> is deprecated: use uuv_plume_msgs-srv:SetPlumeConfiguration-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetPlumeConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:success-val is deprecated.  Use uuv_plume_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPlumeConfiguration-response>) ostream)
  "Serializes a message object of type '<SetPlumeConfiguration-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPlumeConfiguration-response>) istream)
  "Deserializes a message object of type '<SetPlumeConfiguration-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPlumeConfiguration-response>)))
  "Returns string type for a service object of type '<SetPlumeConfiguration-response>"
  "uuv_plume_msgs/SetPlumeConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPlumeConfiguration-response)))
  "Returns string type for a service object of type 'SetPlumeConfiguration-response"
  "uuv_plume_msgs/SetPlumeConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPlumeConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetPlumeConfiguration-response>"
  "55045be54c5c1ddb03ae4cfc1750c6aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPlumeConfiguration-response)))
  "Returns md5sum for a message object of type 'SetPlumeConfiguration-response"
  "55045be54c5c1ddb03ae4cfc1750c6aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPlumeConfiguration-response>)))
  "Returns full string definition for message of type '<SetPlumeConfiguration-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPlumeConfiguration-response)))
  "Returns full string definition for message of type 'SetPlumeConfiguration-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPlumeConfiguration-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPlumeConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPlumeConfiguration-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPlumeConfiguration)))
  'SetPlumeConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPlumeConfiguration)))
  'SetPlumeConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPlumeConfiguration)))
  "Returns string type for a service object of type '<SetPlumeConfiguration>"
  "uuv_plume_msgs/SetPlumeConfiguration")