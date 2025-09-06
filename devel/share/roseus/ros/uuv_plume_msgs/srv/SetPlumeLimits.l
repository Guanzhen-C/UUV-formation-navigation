;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::SetPlumeLimits)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'SetPlumeLimits (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMELIMITS")
  (make-package "UUV_PLUME_MSGS::SETPLUMELIMITS"))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMELIMITSREQUEST")
  (make-package "UUV_PLUME_MSGS::SETPLUMELIMITSREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMELIMITSRESPONSE")
  (make-package "UUV_PLUME_MSGS::SETPLUMELIMITSRESPONSE"))

(in-package "ROS")





(defclass uuv_plume_msgs::SetPlumeLimitsRequest
  :super ros::object
  :slots (_x_min _x_max _y_min _y_max _z_min _z_max ))

(defmethod uuv_plume_msgs::SetPlumeLimitsRequest
  (:init
   (&key
    ((:x_min __x_min) 0.0)
    ((:x_max __x_max) 0.0)
    ((:y_min __y_min) 0.0)
    ((:y_max __y_max) 0.0)
    ((:z_min __z_min) 0.0)
    ((:z_max __z_max) 0.0)
    )
   (send-super :init)
   (setq _x_min (float __x_min))
   (setq _x_max (float __x_max))
   (setq _y_min (float __y_min))
   (setq _y_max (float __y_max))
   (setq _z_min (float __z_min))
   (setq _z_max (float __z_max))
   self)
  (:x_min
   (&optional __x_min)
   (if __x_min (setq _x_min __x_min)) _x_min)
  (:x_max
   (&optional __x_max)
   (if __x_max (setq _x_max __x_max)) _x_max)
  (:y_min
   (&optional __y_min)
   (if __y_min (setq _y_min __y_min)) _y_min)
  (:y_max
   (&optional __y_max)
   (if __y_max (setq _y_max __y_max)) _y_max)
  (:z_min
   (&optional __z_min)
   (if __z_min (setq _z_min __z_min)) _z_min)
  (:z_max
   (&optional __z_max)
   (if __z_max (setq _z_max __z_max)) _z_max)
  (:serialization-length
   ()
   (+
    ;; float64 _x_min
    8
    ;; float64 _x_max
    8
    ;; float64 _y_min
    8
    ;; float64 _y_max
    8
    ;; float64 _z_min
    8
    ;; float64 _z_max
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_min
       (sys::poke _x_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x_max
       (sys::poke _x_max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_min
       (sys::poke _y_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_max
       (sys::poke _y_max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_min
       (sys::poke _z_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_max
       (sys::poke _z_max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_min
     (setq _x_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x_max
     (setq _x_max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_min
     (setq _y_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_max
     (setq _y_max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_min
     (setq _z_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_max
     (setq _z_max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_plume_msgs::SetPlumeLimitsResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_plume_msgs::SetPlumeLimitsResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass uuv_plume_msgs::SetPlumeLimits
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::SetPlumeLimits :md5sum-) "e7030c8a2a5eb00a0d6cf898df109eb8")
(setf (get uuv_plume_msgs::SetPlumeLimits :datatype-) "uuv_plume_msgs/SetPlumeLimits")
(setf (get uuv_plume_msgs::SetPlumeLimits :request) uuv_plume_msgs::SetPlumeLimitsRequest)
(setf (get uuv_plume_msgs::SetPlumeLimits :response) uuv_plume_msgs::SetPlumeLimitsResponse)

(defmethod uuv_plume_msgs::SetPlumeLimitsRequest
  (:response () (instance uuv_plume_msgs::SetPlumeLimitsResponse :init)))

(setf (get uuv_plume_msgs::SetPlumeLimitsRequest :md5sum-) "e7030c8a2a5eb00a0d6cf898df109eb8")
(setf (get uuv_plume_msgs::SetPlumeLimitsRequest :datatype-) "uuv_plume_msgs/SetPlumeLimitsRequest")
(setf (get uuv_plume_msgs::SetPlumeLimitsRequest :definition-)
      "# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max
---
bool success

")

(setf (get uuv_plume_msgs::SetPlumeLimitsResponse :md5sum-) "e7030c8a2a5eb00a0d6cf898df109eb8")
(setf (get uuv_plume_msgs::SetPlumeLimitsResponse :datatype-) "uuv_plume_msgs/SetPlumeLimitsResponse")
(setf (get uuv_plume_msgs::SetPlumeLimitsResponse :definition-)
      "# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the \"License\");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max
---
bool success

")



(provide :uuv_plume_msgs/SetPlumeLimits "e7030c8a2a5eb00a0d6cf898df109eb8")


