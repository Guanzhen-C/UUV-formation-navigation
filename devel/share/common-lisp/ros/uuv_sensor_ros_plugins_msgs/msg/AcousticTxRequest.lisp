; Auto-generated. Do not edit!


(cl:in-package uuv_sensor_ros_plugins_msgs-msg)


;//! \htmlinclude AcousticTxRequest.msg.html

(cl:defclass <AcousticTxRequest> (roslisp-msg-protocol:ros-message)
  ((target_ns
    :reader target_ns
    :initarg :target_ns
    :type cl:string
    :initform "")
   (request_id
    :reader request_id
    :initarg :request_id
    :type cl:string
    :initform ""))
)

(cl:defclass AcousticTxRequest (<AcousticTxRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AcousticTxRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AcousticTxRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_sensor_ros_plugins_msgs-msg:<AcousticTxRequest> is deprecated: use uuv_sensor_ros_plugins_msgs-msg:AcousticTxRequest instead.")))

(cl:ensure-generic-function 'target_ns-val :lambda-list '(m))
(cl:defmethod target_ns-val ((m <AcousticTxRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:target_ns-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:target_ns instead.")
  (target_ns m))

(cl:ensure-generic-function 'request_id-val :lambda-list '(m))
(cl:defmethod request_id-val ((m <AcousticTxRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_sensor_ros_plugins_msgs-msg:request_id-val is deprecated.  Use uuv_sensor_ros_plugins_msgs-msg:request_id instead.")
  (request_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AcousticTxRequest>) ostream)
  "Serializes a message object of type '<AcousticTxRequest>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target_ns))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target_ns))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AcousticTxRequest>) istream)
  "Deserializes a message object of type '<AcousticTxRequest>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_ns) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target_ns) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AcousticTxRequest>)))
  "Returns string type for a message object of type '<AcousticTxRequest>"
  "uuv_sensor_ros_plugins_msgs/AcousticTxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AcousticTxRequest)))
  "Returns string type for a message object of type 'AcousticTxRequest"
  "uuv_sensor_ros_plugins_msgs/AcousticTxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AcousticTxRequest>)))
  "Returns md5sum for a message object of type '<AcousticTxRequest>"
  "c1b001fcc36be9a459785df486239119")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AcousticTxRequest)))
  "Returns md5sum for a message object of type 'AcousticTxRequest"
  "c1b001fcc36be9a459785df486239119")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AcousticTxRequest>)))
  "Returns full string definition for message of type '<AcousticTxRequest>"
  (cl:format cl:nil "# Target AUV namespace to contact (e.g., eca_a9_2)~%string target_ns~%# Optional request identifier for bookkeeping~%string request_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AcousticTxRequest)))
  "Returns full string definition for message of type 'AcousticTxRequest"
  (cl:format cl:nil "# Target AUV namespace to contact (e.g., eca_a9_2)~%string target_ns~%# Optional request identifier for bookkeeping~%string request_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AcousticTxRequest>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'target_ns))
     4 (cl:length (cl:slot-value msg 'request_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AcousticTxRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'AcousticTxRequest
    (cl:cons ':target_ns (target_ns msg))
    (cl:cons ':request_id (request_id msg))
))
