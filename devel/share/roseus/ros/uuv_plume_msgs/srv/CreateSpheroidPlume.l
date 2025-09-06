;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::CreateSpheroidPlume)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'CreateSpheroidPlume (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::CREATESPHEROIDPLUME")
  (make-package "UUV_PLUME_MSGS::CREATESPHEROIDPLUME"))
(unless (find-package "UUV_PLUME_MSGS::CREATESPHEROIDPLUMEREQUEST")
  (make-package "UUV_PLUME_MSGS::CREATESPHEROIDPLUMEREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::CREATESPHEROIDPLUMERESPONSE")
  (make-package "UUV_PLUME_MSGS::CREATESPHEROIDPLUMERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass uuv_plume_msgs::CreateSpheroidPlumeRequest
  :super ros::object
  :slots (_source _orientation _n_points _a _c _x_min _x_max _y_min _y_max _z_min _z_max ))

(defmethod uuv_plume_msgs::CreateSpheroidPlumeRequest
  (:init
   (&key
    ((:source __source) (instance geometry_msgs::Point :init))
    ((:orientation __orientation) (instance geometry_msgs::Quaternion :init))
    ((:n_points __n_points) 0)
    ((:a __a) 0.0)
    ((:c __c) 0.0)
    ((:x_min __x_min) 0.0)
    ((:x_max __x_max) 0.0)
    ((:y_min __y_min) 0.0)
    ((:y_max __y_max) 0.0)
    ((:z_min __z_min) 0.0)
    ((:z_max __z_max) 0.0)
    )
   (send-super :init)
   (setq _source __source)
   (setq _orientation __orientation)
   (setq _n_points (round __n_points))
   (setq _a (float __a))
   (setq _c (float __c))
   (setq _x_min (float __x_min))
   (setq _x_max (float __x_max))
   (setq _y_min (float __y_min))
   (setq _y_max (float __y_max))
   (setq _z_min (float __z_min))
   (setq _z_max (float __z_max))
   self)
  (:source
   (&rest __source)
   (if (keywordp (car __source))
       (send* _source __source)
     (progn
       (if __source (setq _source (car __source)))
       _source)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:n_points
   (&optional __n_points)
   (if __n_points (setq _n_points __n_points)) _n_points)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:c
   (&optional __c)
   (if __c (setq _c __c)) _c)
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
    ;; geometry_msgs/Point _source
    (send _source :serialization-length)
    ;; geometry_msgs/Quaternion _orientation
    (send _orientation :serialization-length)
    ;; int32 _n_points
    4
    ;; float32 _a
    4
    ;; float32 _c
    4
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
     ;; geometry_msgs/Point _source
       (send _source :serialize s)
     ;; geometry_msgs/Quaternion _orientation
       (send _orientation :serialize s)
     ;; int32 _n_points
       (write-long _n_points s)
     ;; float32 _a
       (sys::poke _a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _c
       (sys::poke _c (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; geometry_msgs/Point _source
     (send _source :deserialize buf ptr-) (incf ptr- (send _source :serialization-length))
   ;; geometry_msgs/Quaternion _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;; int32 _n_points
     (setq _n_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _a
     (setq _a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _c
     (setq _c (sys::peek buf ptr- :float)) (incf ptr- 4)
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

(defclass uuv_plume_msgs::CreateSpheroidPlumeResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_plume_msgs::CreateSpheroidPlumeResponse
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

(defclass uuv_plume_msgs::CreateSpheroidPlume
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::CreateSpheroidPlume :md5sum-) "38f854ae91d626d8d19fa91958cef8f9")
(setf (get uuv_plume_msgs::CreateSpheroidPlume :datatype-) "uuv_plume_msgs/CreateSpheroidPlume")
(setf (get uuv_plume_msgs::CreateSpheroidPlume :request) uuv_plume_msgs::CreateSpheroidPlumeRequest)
(setf (get uuv_plume_msgs::CreateSpheroidPlume :response) uuv_plume_msgs::CreateSpheroidPlumeResponse)

(defmethod uuv_plume_msgs::CreateSpheroidPlumeRequest
  (:response () (instance uuv_plume_msgs::CreateSpheroidPlumeResponse :init)))

(setf (get uuv_plume_msgs::CreateSpheroidPlumeRequest :md5sum-) "38f854ae91d626d8d19fa91958cef8f9")
(setf (get uuv_plume_msgs::CreateSpheroidPlumeRequest :datatype-) "uuv_plume_msgs/CreateSpheroidPlumeRequest")
(setf (get uuv_plume_msgs::CreateSpheroidPlumeRequest :definition-)
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

geometry_msgs/Point source
geometry_msgs/Quaternion orientation
int32 n_points
float32 a
float32 c
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max

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
---
bool success

")

(setf (get uuv_plume_msgs::CreateSpheroidPlumeResponse :md5sum-) "38f854ae91d626d8d19fa91958cef8f9")
(setf (get uuv_plume_msgs::CreateSpheroidPlumeResponse :datatype-) "uuv_plume_msgs/CreateSpheroidPlumeResponse")
(setf (get uuv_plume_msgs::CreateSpheroidPlumeResponse :definition-)
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

geometry_msgs/Point source
geometry_msgs/Quaternion orientation
int32 n_points
float32 a
float32 c
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max

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
---
bool success

")



(provide :uuv_plume_msgs/CreateSpheroidPlume "38f854ae91d626d8d19fa91958cef8f9")


