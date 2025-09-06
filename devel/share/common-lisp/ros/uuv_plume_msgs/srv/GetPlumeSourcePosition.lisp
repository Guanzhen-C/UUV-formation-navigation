; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-srv)


;//! \htmlinclude GetPlumeSourcePosition-request.msg.html

(cl:defclass <GetPlumeSourcePosition-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetPlumeSourcePosition-request (<GetPlumeSourcePosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPlumeSourcePosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPlumeSourcePosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<GetPlumeSourcePosition-request> is deprecated: use uuv_plume_msgs-srv:GetPlumeSourcePosition-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPlumeSourcePosition-request>) ostream)
  "Serializes a message object of type '<GetPlumeSourcePosition-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPlumeSourcePosition-request>) istream)
  "Deserializes a message object of type '<GetPlumeSourcePosition-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPlumeSourcePosition-request>)))
  "Returns string type for a service object of type '<GetPlumeSourcePosition-request>"
  "uuv_plume_msgs/GetPlumeSourcePositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPlumeSourcePosition-request)))
  "Returns string type for a service object of type 'GetPlumeSourcePosition-request"
  "uuv_plume_msgs/GetPlumeSourcePositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPlumeSourcePosition-request>)))
  "Returns md5sum for a message object of type '<GetPlumeSourcePosition-request>"
  "74e696b4bb2fd623e3aef7e08928baee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPlumeSourcePosition-request)))
  "Returns md5sum for a message object of type 'GetPlumeSourcePosition-request"
  "74e696b4bb2fd623e3aef7e08928baee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPlumeSourcePosition-request>)))
  "Returns full string definition for message of type '<GetPlumeSourcePosition-request>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPlumeSourcePosition-request)))
  "Returns full string definition for message of type 'GetPlumeSourcePosition-request"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPlumeSourcePosition-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPlumeSourcePosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPlumeSourcePosition-request
))
;//! \htmlinclude GetPlumeSourcePosition-response.msg.html

(cl:defclass <GetPlumeSourcePosition-response> (roslisp-msg-protocol:ros-message)
  ((source
    :reader source
    :initarg :source
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass GetPlumeSourcePosition-response (<GetPlumeSourcePosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPlumeSourcePosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPlumeSourcePosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<GetPlumeSourcePosition-response> is deprecated: use uuv_plume_msgs-srv:GetPlumeSourcePosition-response instead.")))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <GetPlumeSourcePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:source-val is deprecated.  Use uuv_plume_msgs-srv:source instead.")
  (source m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPlumeSourcePosition-response>) ostream)
  "Serializes a message object of type '<GetPlumeSourcePosition-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'source) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPlumeSourcePosition-response>) istream)
  "Deserializes a message object of type '<GetPlumeSourcePosition-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'source) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPlumeSourcePosition-response>)))
  "Returns string type for a service object of type '<GetPlumeSourcePosition-response>"
  "uuv_plume_msgs/GetPlumeSourcePositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPlumeSourcePosition-response)))
  "Returns string type for a service object of type 'GetPlumeSourcePosition-response"
  "uuv_plume_msgs/GetPlumeSourcePositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPlumeSourcePosition-response>)))
  "Returns md5sum for a message object of type '<GetPlumeSourcePosition-response>"
  "74e696b4bb2fd623e3aef7e08928baee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPlumeSourcePosition-response)))
  "Returns md5sum for a message object of type 'GetPlumeSourcePosition-response"
  "74e696b4bb2fd623e3aef7e08928baee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPlumeSourcePosition-response>)))
  "Returns full string definition for message of type '<GetPlumeSourcePosition-response>"
  (cl:format cl:nil "geometry_msgs/Point source~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPlumeSourcePosition-response)))
  "Returns full string definition for message of type 'GetPlumeSourcePosition-response"
  (cl:format cl:nil "geometry_msgs/Point source~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPlumeSourcePosition-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'source))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPlumeSourcePosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPlumeSourcePosition-response
    (cl:cons ':source (source msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPlumeSourcePosition)))
  'GetPlumeSourcePosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPlumeSourcePosition)))
  'GetPlumeSourcePosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPlumeSourcePosition)))
  "Returns string type for a service object of type '<GetPlumeSourcePosition>"
  "uuv_plume_msgs/GetPlumeSourcePosition")