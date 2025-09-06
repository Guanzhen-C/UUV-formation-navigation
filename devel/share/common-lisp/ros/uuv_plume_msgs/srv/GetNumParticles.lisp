; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-srv)


;//! \htmlinclude GetNumParticles-request.msg.html

(cl:defclass <GetNumParticles-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetNumParticles-request (<GetNumParticles-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetNumParticles-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetNumParticles-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<GetNumParticles-request> is deprecated: use uuv_plume_msgs-srv:GetNumParticles-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetNumParticles-request>) ostream)
  "Serializes a message object of type '<GetNumParticles-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetNumParticles-request>) istream)
  "Deserializes a message object of type '<GetNumParticles-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetNumParticles-request>)))
  "Returns string type for a service object of type '<GetNumParticles-request>"
  "uuv_plume_msgs/GetNumParticlesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetNumParticles-request)))
  "Returns string type for a service object of type 'GetNumParticles-request"
  "uuv_plume_msgs/GetNumParticlesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetNumParticles-request>)))
  "Returns md5sum for a message object of type '<GetNumParticles-request>"
  "b3e7a05396001d2e02f04b13547a3da2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetNumParticles-request)))
  "Returns md5sum for a message object of type 'GetNumParticles-request"
  "b3e7a05396001d2e02f04b13547a3da2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetNumParticles-request>)))
  "Returns full string definition for message of type '<GetNumParticles-request>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetNumParticles-request)))
  "Returns full string definition for message of type 'GetNumParticles-request"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetNumParticles-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetNumParticles-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetNumParticles-request
))
;//! \htmlinclude GetNumParticles-response.msg.html

(cl:defclass <GetNumParticles-response> (roslisp-msg-protocol:ros-message)
  ((num_particles
    :reader num_particles
    :initarg :num_particles
    :type cl:integer
    :initform 0))
)

(cl:defclass GetNumParticles-response (<GetNumParticles-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetNumParticles-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetNumParticles-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<GetNumParticles-response> is deprecated: use uuv_plume_msgs-srv:GetNumParticles-response instead.")))

(cl:ensure-generic-function 'num_particles-val :lambda-list '(m))
(cl:defmethod num_particles-val ((m <GetNumParticles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:num_particles-val is deprecated.  Use uuv_plume_msgs-srv:num_particles instead.")
  (num_particles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetNumParticles-response>) ostream)
  "Serializes a message object of type '<GetNumParticles-response>"
  (cl:let* ((signed (cl:slot-value msg 'num_particles)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetNumParticles-response>) istream)
  "Deserializes a message object of type '<GetNumParticles-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_particles) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetNumParticles-response>)))
  "Returns string type for a service object of type '<GetNumParticles-response>"
  "uuv_plume_msgs/GetNumParticlesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetNumParticles-response)))
  "Returns string type for a service object of type 'GetNumParticles-response"
  "uuv_plume_msgs/GetNumParticlesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetNumParticles-response>)))
  "Returns md5sum for a message object of type '<GetNumParticles-response>"
  "b3e7a05396001d2e02f04b13547a3da2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetNumParticles-response)))
  "Returns md5sum for a message object of type 'GetNumParticles-response"
  "b3e7a05396001d2e02f04b13547a3da2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetNumParticles-response>)))
  "Returns full string definition for message of type '<GetNumParticles-response>"
  (cl:format cl:nil "int32 num_particles~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetNumParticles-response)))
  "Returns full string definition for message of type 'GetNumParticles-response"
  (cl:format cl:nil "int32 num_particles~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetNumParticles-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetNumParticles-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetNumParticles-response
    (cl:cons ':num_particles (num_particles msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetNumParticles)))
  'GetNumParticles-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetNumParticles)))
  'GetNumParticles-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetNumParticles)))
  "Returns string type for a service object of type '<GetNumParticles>"
  "uuv_plume_msgs/GetNumParticles")