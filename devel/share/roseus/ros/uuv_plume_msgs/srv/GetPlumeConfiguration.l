;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::GetPlumeConfiguration)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'GetPlumeConfiguration (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::GETPLUMECONFIGURATION")
  (make-package "UUV_PLUME_MSGS::GETPLUMECONFIGURATION"))
(unless (find-package "UUV_PLUME_MSGS::GETPLUMECONFIGURATIONREQUEST")
  (make-package "UUV_PLUME_MSGS::GETPLUMECONFIGURATIONREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::GETPLUMECONFIGURATIONRESPONSE")
  (make-package "UUV_PLUME_MSGS::GETPLUMECONFIGURATIONRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uuv_plume_msgs::GetPlumeConfigurationRequest
  :super ros::object
  :slots ())

(defmethod uuv_plume_msgs::GetPlumeConfigurationRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass uuv_plume_msgs::GetPlumeConfigurationResponse
  :super ros::object
  :slots (_model_name _n_points _max_particles_per_iter _source _x_min _x_max _y_min _y_max _z_min _z_max ))

(defmethod uuv_plume_msgs::GetPlumeConfigurationResponse
  (:init
   (&key
    ((:model_name __model_name) "")
    ((:n_points __n_points) 0)
    ((:max_particles_per_iter __max_particles_per_iter) 0)
    ((:source __source) (instance geometry_msgs::Point :init))
    ((:x_min __x_min) 0.0)
    ((:x_max __x_max) 0.0)
    ((:y_min __y_min) 0.0)
    ((:y_max __y_max) 0.0)
    ((:z_min __z_min) 0.0)
    ((:z_max __z_max) 0.0)
    )
   (send-super :init)
   (setq _model_name (string __model_name))
   (setq _n_points (round __n_points))
   (setq _max_particles_per_iter (round __max_particles_per_iter))
   (setq _source __source)
   (setq _x_min (float __x_min))
   (setq _x_max (float __x_max))
   (setq _y_min (float __y_min))
   (setq _y_max (float __y_max))
   (setq _z_min (float __z_min))
   (setq _z_max (float __z_max))
   self)
  (:model_name
   (&optional __model_name)
   (if __model_name (setq _model_name __model_name)) _model_name)
  (:n_points
   (&optional __n_points)
   (if __n_points (setq _n_points __n_points)) _n_points)
  (:max_particles_per_iter
   (&optional __max_particles_per_iter)
   (if __max_particles_per_iter (setq _max_particles_per_iter __max_particles_per_iter)) _max_particles_per_iter)
  (:source
   (&rest __source)
   (if (keywordp (car __source))
       (send* _source __source)
     (progn
       (if __source (setq _source (car __source)))
       _source)))
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
    ;; string _model_name
    4 (length _model_name)
    ;; int32 _n_points
    4
    ;; int32 _max_particles_per_iter
    4
    ;; geometry_msgs/Point _source
    (send _source :serialization-length)
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
     ;; string _model_name
       (write-long (length _model_name) s) (princ _model_name s)
     ;; int32 _n_points
       (write-long _n_points s)
     ;; int32 _max_particles_per_iter
       (write-long _max_particles_per_iter s)
     ;; geometry_msgs/Point _source
       (send _source :serialize s)
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
   ;; string _model_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _n_points
     (setq _n_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_particles_per_iter
     (setq _max_particles_per_iter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point _source
     (send _source :deserialize buf ptr-) (incf ptr- (send _source :serialization-length))
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

(defclass uuv_plume_msgs::GetPlumeConfiguration
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::GetPlumeConfiguration :md5sum-) "eb1615b61ec9b80c7981906c8ffbab07")
(setf (get uuv_plume_msgs::GetPlumeConfiguration :datatype-) "uuv_plume_msgs/GetPlumeConfiguration")
(setf (get uuv_plume_msgs::GetPlumeConfiguration :request) uuv_plume_msgs::GetPlumeConfigurationRequest)
(setf (get uuv_plume_msgs::GetPlumeConfiguration :response) uuv_plume_msgs::GetPlumeConfigurationResponse)

(defmethod uuv_plume_msgs::GetPlumeConfigurationRequest
  (:response () (instance uuv_plume_msgs::GetPlumeConfigurationResponse :init)))

(setf (get uuv_plume_msgs::GetPlumeConfigurationRequest :md5sum-) "eb1615b61ec9b80c7981906c8ffbab07")
(setf (get uuv_plume_msgs::GetPlumeConfigurationRequest :datatype-) "uuv_plume_msgs/GetPlumeConfigurationRequest")
(setf (get uuv_plume_msgs::GetPlumeConfigurationRequest :definition-)
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

---
string model_name
int32 n_points
int32 max_particles_per_iter
geometry_msgs/Point source
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
")

(setf (get uuv_plume_msgs::GetPlumeConfigurationResponse :md5sum-) "eb1615b61ec9b80c7981906c8ffbab07")
(setf (get uuv_plume_msgs::GetPlumeConfigurationResponse :datatype-) "uuv_plume_msgs/GetPlumeConfigurationResponse")
(setf (get uuv_plume_msgs::GetPlumeConfigurationResponse :definition-)
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

---
string model_name
int32 n_points
int32 max_particles_per_iter
geometry_msgs/Point source
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
")



(provide :uuv_plume_msgs/GetPlumeConfiguration "eb1615b61ec9b80c7981906c8ffbab07")


