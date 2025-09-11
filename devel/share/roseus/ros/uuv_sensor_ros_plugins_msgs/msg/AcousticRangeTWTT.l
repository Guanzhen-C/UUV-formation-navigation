;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT)
  (if (not (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS"))
    (make-package "UUV_SENSOR_ROS_PLUGINS_MSGS"))
  (shadow 'AcousticRangeTWTT (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS::ACOUSTICRANGETWTT")
  (make-package "UUV_SENSOR_ROS_PLUGINS_MSGS::ACOUSTICRANGETWTT"))

(in-package "ROS")
;;//! \htmlinclude AcousticRangeTWTT.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT
  :super ros::object
  :slots (_from_ns _to_ns _t0 _t1 _t2 _r1 _peer_pose_t1 _success _variance ))

(defmethod uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT
  (:init
   (&key
    ((:from_ns __from_ns) "")
    ((:to_ns __to_ns) "")
    ((:t0 __t0) (instance ros::time :init))
    ((:t1 __t1) (instance ros::time :init))
    ((:t2 __t2) (instance ros::time :init))
    ((:r1 __r1) 0.0)
    ((:peer_pose_t1 __peer_pose_t1) (instance geometry_msgs::PoseStamped :init))
    ((:success __success) nil)
    ((:variance __variance) 0.0)
    )
   (send-super :init)
   (setq _from_ns (string __from_ns))
   (setq _to_ns (string __to_ns))
   (setq _t0 __t0)
   (setq _t1 __t1)
   (setq _t2 __t2)
   (setq _r1 (float __r1))
   (setq _peer_pose_t1 __peer_pose_t1)
   (setq _success __success)
   (setq _variance (float __variance))
   self)
  (:from_ns
   (&optional __from_ns)
   (if __from_ns (setq _from_ns __from_ns)) _from_ns)
  (:to_ns
   (&optional __to_ns)
   (if __to_ns (setq _to_ns __to_ns)) _to_ns)
  (:t0
   (&optional __t0)
   (if __t0 (setq _t0 __t0)) _t0)
  (:t1
   (&optional __t1)
   (if __t1 (setq _t1 __t1)) _t1)
  (:t2
   (&optional __t2)
   (if __t2 (setq _t2 __t2)) _t2)
  (:r1
   (&optional __r1)
   (if __r1 (setq _r1 __r1)) _r1)
  (:peer_pose_t1
   (&rest __peer_pose_t1)
   (if (keywordp (car __peer_pose_t1))
       (send* _peer_pose_t1 __peer_pose_t1)
     (progn
       (if __peer_pose_t1 (setq _peer_pose_t1 (car __peer_pose_t1)))
       _peer_pose_t1)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:variance
   (&optional __variance)
   (if __variance (setq _variance __variance)) _variance)
  (:serialization-length
   ()
   (+
    ;; string _from_ns
    4 (length _from_ns)
    ;; string _to_ns
    4 (length _to_ns)
    ;; time _t0
    8
    ;; time _t1
    8
    ;; time _t2
    8
    ;; float64 _r1
    8
    ;; geometry_msgs/PoseStamped _peer_pose_t1
    (send _peer_pose_t1 :serialization-length)
    ;; bool _success
    1
    ;; float64 _variance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _from_ns
       (write-long (length _from_ns) s) (princ _from_ns s)
     ;; string _to_ns
       (write-long (length _to_ns) s) (princ _to_ns s)
     ;; time _t0
       (write-long (send _t0 :sec) s) (write-long (send _t0 :nsec) s)
     ;; time _t1
       (write-long (send _t1 :sec) s) (write-long (send _t1 :nsec) s)
     ;; time _t2
       (write-long (send _t2 :sec) s) (write-long (send _t2 :nsec) s)
     ;; float64 _r1
       (sys::poke _r1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/PoseStamped _peer_pose_t1
       (send _peer_pose_t1 :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; float64 _variance
       (sys::poke _variance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _from_ns
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _from_ns (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _to_ns
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _to_ns (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _t0
     (send _t0 :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t0 :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _t1
     (send _t1 :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t1 :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _t2
     (send _t2 :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t2 :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _r1
     (setq _r1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/PoseStamped _peer_pose_t1
     (send _peer_pose_t1 :deserialize buf ptr-) (incf ptr- (send _peer_pose_t1 :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _variance
     (setq _variance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT :md5sum-) "998a5403ceadaee45594f77aee3f1e98")
(setf (get uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT :datatype-) "uuv_sensor_ros_plugins_msgs/AcousticRangeTWTT")
(setf (get uuv_sensor_ros_plugins_msgs::AcousticRangeTWTT :definition-)
      "# From initiator namespace to responder namespace
string from_ns
string to_ns
# Time stamps in ROS time (sim time)
time t0
# arrival time at responder (unknown to initiator; for logging)
time t1
# arrival time of reply at initiator
time t2
# Measured r1 = || p1(t2) - p2(t1) || (delta assumed 0 in minimal version)
float64 r1
# Peer pose at t1 (as provided by responder)
geometry_msgs/PoseStamped peer_pose_t1
# Success flag and variance of r1
bool success
float64 variance

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :uuv_sensor_ros_plugins_msgs/AcousticRangeTWTT "998a5403ceadaee45594f77aee3f1e98")


