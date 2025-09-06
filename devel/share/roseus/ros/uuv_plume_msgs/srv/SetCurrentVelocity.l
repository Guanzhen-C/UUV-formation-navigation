;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::SetCurrentVelocity)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'SetCurrentVelocity (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::SETCURRENTVELOCITY")
  (make-package "UUV_PLUME_MSGS::SETCURRENTVELOCITY"))
(unless (find-package "UUV_PLUME_MSGS::SETCURRENTVELOCITYREQUEST")
  (make-package "UUV_PLUME_MSGS::SETCURRENTVELOCITYREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::SETCURRENTVELOCITYRESPONSE")
  (make-package "UUV_PLUME_MSGS::SETCURRENTVELOCITYRESPONSE"))

(in-package "ROS")





(defclass uuv_plume_msgs::SetCurrentVelocityRequest
  :super ros::object
  :slots (_velocity _horizontal_angle ))

(defmethod uuv_plume_msgs::SetCurrentVelocityRequest
  (:init
   (&key
    ((:velocity __velocity) 0.0)
    ((:horizontal_angle __horizontal_angle) 0.0)
    )
   (send-super :init)
   (setq _velocity (float __velocity))
   (setq _horizontal_angle (float __horizontal_angle))
   self)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:horizontal_angle
   (&optional __horizontal_angle)
   (if __horizontal_angle (setq _horizontal_angle __horizontal_angle)) _horizontal_angle)
  (:serialization-length
   ()
   (+
    ;; float64 _velocity
    8
    ;; float64 _horizontal_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _horizontal_angle
       (sys::poke _horizontal_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _horizontal_angle
     (setq _horizontal_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_plume_msgs::SetCurrentVelocityResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_plume_msgs::SetCurrentVelocityResponse
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

(defclass uuv_plume_msgs::SetCurrentVelocity
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::SetCurrentVelocity :md5sum-) "29979ce59631380ff17a987b6ff491eb")
(setf (get uuv_plume_msgs::SetCurrentVelocity :datatype-) "uuv_plume_msgs/SetCurrentVelocity")
(setf (get uuv_plume_msgs::SetCurrentVelocity :request) uuv_plume_msgs::SetCurrentVelocityRequest)
(setf (get uuv_plume_msgs::SetCurrentVelocity :response) uuv_plume_msgs::SetCurrentVelocityResponse)

(defmethod uuv_plume_msgs::SetCurrentVelocityRequest
  (:response () (instance uuv_plume_msgs::SetCurrentVelocityResponse :init)))

(setf (get uuv_plume_msgs::SetCurrentVelocityRequest :md5sum-) "29979ce59631380ff17a987b6ff491eb")
(setf (get uuv_plume_msgs::SetCurrentVelocityRequest :datatype-) "uuv_plume_msgs/SetCurrentVelocityRequest")
(setf (get uuv_plume_msgs::SetCurrentVelocityRequest :definition-)
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

float64 velocity
float64 horizontal_angle
---
bool success

")

(setf (get uuv_plume_msgs::SetCurrentVelocityResponse :md5sum-) "29979ce59631380ff17a987b6ff491eb")
(setf (get uuv_plume_msgs::SetCurrentVelocityResponse :datatype-) "uuv_plume_msgs/SetCurrentVelocityResponse")
(setf (get uuv_plume_msgs::SetCurrentVelocityResponse :definition-)
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

float64 velocity
float64 horizontal_angle
---
bool success

")



(provide :uuv_plume_msgs/SetCurrentVelocity "29979ce59631380ff17a987b6ff491eb")


