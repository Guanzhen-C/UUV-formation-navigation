; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-srv)


;//! \htmlinclude StorePlumeState-request.msg.html

(cl:defclass <StorePlumeState-request> (roslisp-msg-protocol:ros-message)
  ((output_dir
    :reader output_dir
    :initarg :output_dir
    :type cl:string
    :initform "")
   (filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass StorePlumeState-request (<StorePlumeState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StorePlumeState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StorePlumeState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<StorePlumeState-request> is deprecated: use uuv_plume_msgs-srv:StorePlumeState-request instead.")))

(cl:ensure-generic-function 'output_dir-val :lambda-list '(m))
(cl:defmethod output_dir-val ((m <StorePlumeState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:output_dir-val is deprecated.  Use uuv_plume_msgs-srv:output_dir instead.")
  (output_dir m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <StorePlumeState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:filename-val is deprecated.  Use uuv_plume_msgs-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StorePlumeState-request>) ostream)
  "Serializes a message object of type '<StorePlumeState-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output_dir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output_dir))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StorePlumeState-request>) istream)
  "Deserializes a message object of type '<StorePlumeState-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'output_dir) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'output_dir) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StorePlumeState-request>)))
  "Returns string type for a service object of type '<StorePlumeState-request>"
  "uuv_plume_msgs/StorePlumeStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StorePlumeState-request)))
  "Returns string type for a service object of type 'StorePlumeState-request"
  "uuv_plume_msgs/StorePlumeStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StorePlumeState-request>)))
  "Returns md5sum for a message object of type '<StorePlumeState-request>"
  "4fbc4c340238072a838f039245e40b09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StorePlumeState-request)))
  "Returns md5sum for a message object of type 'StorePlumeState-request"
  "4fbc4c340238072a838f039245e40b09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StorePlumeState-request>)))
  "Returns full string definition for message of type '<StorePlumeState-request>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string output_dir~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StorePlumeState-request)))
  "Returns full string definition for message of type 'StorePlumeState-request"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%string output_dir~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StorePlumeState-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output_dir))
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StorePlumeState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StorePlumeState-request
    (cl:cons ':output_dir (output_dir msg))
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude StorePlumeState-response.msg.html

(cl:defclass <StorePlumeState-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (plume_file
    :reader plume_file
    :initarg :plume_file
    :type cl:string
    :initform ""))
)

(cl:defclass StorePlumeState-response (<StorePlumeState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StorePlumeState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StorePlumeState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<StorePlumeState-response> is deprecated: use uuv_plume_msgs-srv:StorePlumeState-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StorePlumeState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:success-val is deprecated.  Use uuv_plume_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'plume_file-val :lambda-list '(m))
(cl:defmethod plume_file-val ((m <StorePlumeState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:plume_file-val is deprecated.  Use uuv_plume_msgs-srv:plume_file instead.")
  (plume_file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StorePlumeState-response>) ostream)
  "Serializes a message object of type '<StorePlumeState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'plume_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'plume_file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StorePlumeState-response>) istream)
  "Deserializes a message object of type '<StorePlumeState-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StorePlumeState-response>)))
  "Returns string type for a service object of type '<StorePlumeState-response>"
  "uuv_plume_msgs/StorePlumeStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StorePlumeState-response)))
  "Returns string type for a service object of type 'StorePlumeState-response"
  "uuv_plume_msgs/StorePlumeStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StorePlumeState-response>)))
  "Returns md5sum for a message object of type '<StorePlumeState-response>"
  "4fbc4c340238072a838f039245e40b09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StorePlumeState-response)))
  "Returns md5sum for a message object of type 'StorePlumeState-response"
  "4fbc4c340238072a838f039245e40b09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StorePlumeState-response>)))
  "Returns full string definition for message of type '<StorePlumeState-response>"
  (cl:format cl:nil "bool success~%string plume_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StorePlumeState-response)))
  "Returns full string definition for message of type 'StorePlumeState-response"
  (cl:format cl:nil "bool success~%string plume_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StorePlumeState-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'plume_file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StorePlumeState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StorePlumeState-response
    (cl:cons ':success (success msg))
    (cl:cons ':plume_file (plume_file msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StorePlumeState)))
  'StorePlumeState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StorePlumeState)))
  'StorePlumeState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StorePlumeState)))
  "Returns string type for a service object of type '<StorePlumeState>"
  "uuv_plume_msgs/StorePlumeState")