; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-srv)


;//! \htmlinclude DeletePlume-request.msg.html

(cl:defclass <DeletePlume-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DeletePlume-request (<DeletePlume-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeletePlume-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeletePlume-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<DeletePlume-request> is deprecated: use uuv_plume_msgs-srv:DeletePlume-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeletePlume-request>) ostream)
  "Serializes a message object of type '<DeletePlume-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeletePlume-request>) istream)
  "Deserializes a message object of type '<DeletePlume-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeletePlume-request>)))
  "Returns string type for a service object of type '<DeletePlume-request>"
  "uuv_plume_msgs/DeletePlumeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeletePlume-request)))
  "Returns string type for a service object of type 'DeletePlume-request"
  "uuv_plume_msgs/DeletePlumeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeletePlume-request>)))
  "Returns md5sum for a message object of type '<DeletePlume-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeletePlume-request)))
  "Returns md5sum for a message object of type 'DeletePlume-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeletePlume-request>)))
  "Returns full string definition for message of type '<DeletePlume-request>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeletePlume-request)))
  "Returns full string definition for message of type 'DeletePlume-request"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeletePlume-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeletePlume-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DeletePlume-request
))
;//! \htmlinclude DeletePlume-response.msg.html

(cl:defclass <DeletePlume-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DeletePlume-response (<DeletePlume-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeletePlume-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeletePlume-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<DeletePlume-response> is deprecated: use uuv_plume_msgs-srv:DeletePlume-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DeletePlume-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:success-val is deprecated.  Use uuv_plume_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeletePlume-response>) ostream)
  "Serializes a message object of type '<DeletePlume-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeletePlume-response>) istream)
  "Deserializes a message object of type '<DeletePlume-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeletePlume-response>)))
  "Returns string type for a service object of type '<DeletePlume-response>"
  "uuv_plume_msgs/DeletePlumeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeletePlume-response)))
  "Returns string type for a service object of type 'DeletePlume-response"
  "uuv_plume_msgs/DeletePlumeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeletePlume-response>)))
  "Returns md5sum for a message object of type '<DeletePlume-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeletePlume-response)))
  "Returns md5sum for a message object of type 'DeletePlume-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeletePlume-response>)))
  "Returns full string definition for message of type '<DeletePlume-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeletePlume-response)))
  "Returns full string definition for message of type 'DeletePlume-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeletePlume-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeletePlume-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DeletePlume-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DeletePlume)))
  'DeletePlume-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DeletePlume)))
  'DeletePlume-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeletePlume)))
  "Returns string type for a service object of type '<DeletePlume>"
  "uuv_plume_msgs/DeletePlume")