; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-srv)


;//! \htmlinclude SetPlumeSourcePosition-request.msg.html

(cl:defclass <SetPlumeSourcePosition-request> (roslisp-msg-protocol:ros-message)
  ((source
    :reader source
    :initarg :source
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass SetPlumeSourcePosition-request (<SetPlumeSourcePosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPlumeSourcePosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPlumeSourcePosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<SetPlumeSourcePosition-request> is deprecated: use uuv_plume_msgs-srv:SetPlumeSourcePosition-request instead.")))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <SetPlumeSourcePosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:source-val is deprecated.  Use uuv_plume_msgs-srv:source instead.")
  (source m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPlumeSourcePosition-request>) ostream)
  "Serializes a message object of type '<SetPlumeSourcePosition-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'source) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPlumeSourcePosition-request>) istream)
  "Deserializes a message object of type '<SetPlumeSourcePosition-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'source) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPlumeSourcePosition-request>)))
  "Returns string type for a service object of type '<SetPlumeSourcePosition-request>"
  "uuv_plume_msgs/SetPlumeSourcePositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPlumeSourcePosition-request)))
  "Returns string type for a service object of type 'SetPlumeSourcePosition-request"
  "uuv_plume_msgs/SetPlumeSourcePositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPlumeSourcePosition-request>)))
  "Returns md5sum for a message object of type '<SetPlumeSourcePosition-request>"
  "6f914c566d493481f609db28b34106cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPlumeSourcePosition-request)))
  "Returns md5sum for a message object of type 'SetPlumeSourcePosition-request"
  "6f914c566d493481f609db28b34106cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPlumeSourcePosition-request>)))
  "Returns full string definition for message of type '<SetPlumeSourcePosition-request>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%geometry_msgs/Point source~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPlumeSourcePosition-request)))
  "Returns full string definition for message of type 'SetPlumeSourcePosition-request"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%geometry_msgs/Point source~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPlumeSourcePosition-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'source))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPlumeSourcePosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPlumeSourcePosition-request
    (cl:cons ':source (source msg))
))
;//! \htmlinclude SetPlumeSourcePosition-response.msg.html

(cl:defclass <SetPlumeSourcePosition-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetPlumeSourcePosition-response (<SetPlumeSourcePosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPlumeSourcePosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPlumeSourcePosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<SetPlumeSourcePosition-response> is deprecated: use uuv_plume_msgs-srv:SetPlumeSourcePosition-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetPlumeSourcePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:success-val is deprecated.  Use uuv_plume_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPlumeSourcePosition-response>) ostream)
  "Serializes a message object of type '<SetPlumeSourcePosition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPlumeSourcePosition-response>) istream)
  "Deserializes a message object of type '<SetPlumeSourcePosition-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPlumeSourcePosition-response>)))
  "Returns string type for a service object of type '<SetPlumeSourcePosition-response>"
  "uuv_plume_msgs/SetPlumeSourcePositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPlumeSourcePosition-response)))
  "Returns string type for a service object of type 'SetPlumeSourcePosition-response"
  "uuv_plume_msgs/SetPlumeSourcePositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPlumeSourcePosition-response>)))
  "Returns md5sum for a message object of type '<SetPlumeSourcePosition-response>"
  "6f914c566d493481f609db28b34106cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPlumeSourcePosition-response)))
  "Returns md5sum for a message object of type 'SetPlumeSourcePosition-response"
  "6f914c566d493481f609db28b34106cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPlumeSourcePosition-response>)))
  "Returns full string definition for message of type '<SetPlumeSourcePosition-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPlumeSourcePosition-response)))
  "Returns full string definition for message of type 'SetPlumeSourcePosition-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPlumeSourcePosition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPlumeSourcePosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPlumeSourcePosition-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPlumeSourcePosition)))
  'SetPlumeSourcePosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPlumeSourcePosition)))
  'SetPlumeSourcePosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPlumeSourcePosition)))
  "Returns string type for a service object of type '<SetPlumeSourcePosition>"
  "uuv_plume_msgs/SetPlumeSourcePosition")