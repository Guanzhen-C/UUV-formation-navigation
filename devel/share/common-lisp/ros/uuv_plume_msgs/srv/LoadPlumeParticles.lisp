; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-srv)


;//! \htmlinclude LoadPlumeParticles-request.msg.html

(cl:defclass <LoadPlumeParticles-request> (roslisp-msg-protocol:ros-message)
  ((plume_file
    :reader plume_file
    :initarg :plume_file
    :type cl:string
    :initform ""))
)

(cl:defclass LoadPlumeParticles-request (<LoadPlumeParticles-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadPlumeParticles-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadPlumeParticles-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<LoadPlumeParticles-request> is deprecated: use uuv_plume_msgs-srv:LoadPlumeParticles-request instead.")))

(cl:ensure-generic-function 'plume_file-val :lambda-list '(m))
(cl:defmethod plume_file-val ((m <LoadPlumeParticles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:plume_file-val is deprecated.  Use uuv_plume_msgs-srv:plume_file instead.")
  (plume_file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadPlumeParticles-request>) ostream)
  "Serializes a message object of type '<LoadPlumeParticles-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'plume_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'plume_file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadPlumeParticles-request>) istream)
  "Deserializes a message object of type '<LoadPlumeParticles-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plume_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'plume_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadPlumeParticles-request>)))
  "Returns string type for a service object of type '<LoadPlumeParticles-request>"
  "uuv_plume_msgs/LoadPlumeParticlesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadPlumeParticles-request)))
  "Returns string type for a service object of type 'LoadPlumeParticles-request"
  "uuv_plume_msgs/LoadPlumeParticlesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadPlumeParticles-request>)))
  "Returns md5sum for a message object of type '<LoadPlumeParticles-request>"
  "89cea82104ad560d0ae7df2662f9b983")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadPlumeParticles-request)))
  "Returns md5sum for a message object of type 'LoadPlumeParticles-request"
  "89cea82104ad560d0ae7df2662f9b983")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadPlumeParticles-request>)))
  "Returns full string definition for message of type '<LoadPlumeParticles-request>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string plume_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadPlumeParticles-request)))
  "Returns full string definition for message of type 'LoadPlumeParticles-request"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string plume_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadPlumeParticles-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'plume_file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadPlumeParticles-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadPlumeParticles-request
    (cl:cons ':plume_file (plume_file msg))
))
;//! \htmlinclude LoadPlumeParticles-response.msg.html

(cl:defclass <LoadPlumeParticles-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LoadPlumeParticles-response (<LoadPlumeParticles-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadPlumeParticles-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadPlumeParticles-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<LoadPlumeParticles-response> is deprecated: use uuv_plume_msgs-srv:LoadPlumeParticles-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LoadPlumeParticles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:success-val is deprecated.  Use uuv_plume_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadPlumeParticles-response>) ostream)
  "Serializes a message object of type '<LoadPlumeParticles-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadPlumeParticles-response>) istream)
  "Deserializes a message object of type '<LoadPlumeParticles-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadPlumeParticles-response>)))
  "Returns string type for a service object of type '<LoadPlumeParticles-response>"
  "uuv_plume_msgs/LoadPlumeParticlesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadPlumeParticles-response)))
  "Returns string type for a service object of type 'LoadPlumeParticles-response"
  "uuv_plume_msgs/LoadPlumeParticlesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadPlumeParticles-response>)))
  "Returns md5sum for a message object of type '<LoadPlumeParticles-response>"
  "89cea82104ad560d0ae7df2662f9b983")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadPlumeParticles-response)))
  "Returns md5sum for a message object of type 'LoadPlumeParticles-response"
  "89cea82104ad560d0ae7df2662f9b983")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadPlumeParticles-response>)))
  "Returns full string definition for message of type '<LoadPlumeParticles-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadPlumeParticles-response)))
  "Returns full string definition for message of type 'LoadPlumeParticles-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadPlumeParticles-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadPlumeParticles-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadPlumeParticles-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LoadPlumeParticles)))
  'LoadPlumeParticles-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LoadPlumeParticles)))
  'LoadPlumeParticles-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadPlumeParticles)))
  "Returns string type for a service object of type '<LoadPlumeParticles>"
  "uuv_plume_msgs/LoadPlumeParticles")