;; Auto-generated. Do not edit!


(when (boundp 'uuv_sensor_ros_plugins_msgs::AcousticTxRequest)
  (if (not (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS"))
    (make-package "UUV_SENSOR_ROS_PLUGINS_MSGS"))
  (shadow 'AcousticTxRequest (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS")))
(unless (find-package "UUV_SENSOR_ROS_PLUGINS_MSGS::ACOUSTICTXREQUEST")
  (make-package "UUV_SENSOR_ROS_PLUGINS_MSGS::ACOUSTICTXREQUEST"))

(in-package "ROS")
;;//! \htmlinclude AcousticTxRequest.msg.html


(defclass uuv_sensor_ros_plugins_msgs::AcousticTxRequest
  :super ros::object
  :slots (_target_ns _request_id ))

(defmethod uuv_sensor_ros_plugins_msgs::AcousticTxRequest
  (:init
   (&key
    ((:target_ns __target_ns) "")
    ((:request_id __request_id) "")
    )
   (send-super :init)
   (setq _target_ns (string __target_ns))
   (setq _request_id (string __request_id))
   self)
  (:target_ns
   (&optional __target_ns)
   (if __target_ns (setq _target_ns __target_ns)) _target_ns)
  (:request_id
   (&optional __request_id)
   (if __request_id (setq _request_id __request_id)) _request_id)
  (:serialization-length
   ()
   (+
    ;; string _target_ns
    4 (length _target_ns)
    ;; string _request_id
    4 (length _request_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _target_ns
       (write-long (length _target_ns) s) (princ _target_ns s)
     ;; string _request_id
       (write-long (length _request_id) s) (princ _request_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _target_ns
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target_ns (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _request_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _request_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get uuv_sensor_ros_plugins_msgs::AcousticTxRequest :md5sum-) "c1b001fcc36be9a459785df486239119")
(setf (get uuv_sensor_ros_plugins_msgs::AcousticTxRequest :datatype-) "uuv_sensor_ros_plugins_msgs/AcousticTxRequest")
(setf (get uuv_sensor_ros_plugins_msgs::AcousticTxRequest :definition-)
      "# Target AUV namespace to contact (e.g., eca_a9_2)
string target_ns
# Optional request identifier for bookkeeping
string request_id

")



(provide :uuv_sensor_ros_plugins_msgs/AcousticTxRequest "c1b001fcc36be9a459785df486239119")


