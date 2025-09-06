;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::CreatePassiveScalarTurbulentPlume)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'CreatePassiveScalarTurbulentPlume (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::CREATEPASSIVESCALARTURBULENTPLUME")
  (make-package "UUV_PLUME_MSGS::CREATEPASSIVESCALARTURBULENTPLUME"))
(unless (find-package "UUV_PLUME_MSGS::CREATEPASSIVESCALARTURBULENTPLUMEREQUEST")
  (make-package "UUV_PLUME_MSGS::CREATEPASSIVESCALARTURBULENTPLUMEREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::CREATEPASSIVESCALARTURBULENTPLUMERESPONSE")
  (make-package "UUV_PLUME_MSGS::CREATEPASSIVESCALARTURBULENTPLUMERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeRequest
  :super ros::object
  :slots (_turbulent_diffusion_coefficients _source _buoyancy_flux _stability_param _n_points _max_particles_per_iter _x_min _x_max _y_min _y_max _z_min _z_max _max_life_time ))

(defmethod uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeRequest
  (:init
   (&key
    ((:turbulent_diffusion_coefficients __turbulent_diffusion_coefficients) (instance geometry_msgs::Vector3 :init))
    ((:source __source) (instance geometry_msgs::Point :init))
    ((:buoyancy_flux __buoyancy_flux) 0.0)
    ((:stability_param __stability_param) 0.0)
    ((:n_points __n_points) 0)
    ((:max_particles_per_iter __max_particles_per_iter) 0)
    ((:x_min __x_min) 0.0)
    ((:x_max __x_max) 0.0)
    ((:y_min __y_min) 0.0)
    ((:y_max __y_max) 0.0)
    ((:z_min __z_min) 0.0)
    ((:z_max __z_max) 0.0)
    ((:max_life_time __max_life_time) 0.0)
    )
   (send-super :init)
   (setq _turbulent_diffusion_coefficients __turbulent_diffusion_coefficients)
   (setq _source __source)
   (setq _buoyancy_flux (float __buoyancy_flux))
   (setq _stability_param (float __stability_param))
   (setq _n_points (round __n_points))
   (setq _max_particles_per_iter (round __max_particles_per_iter))
   (setq _x_min (float __x_min))
   (setq _x_max (float __x_max))
   (setq _y_min (float __y_min))
   (setq _y_max (float __y_max))
   (setq _z_min (float __z_min))
   (setq _z_max (float __z_max))
   (setq _max_life_time (float __max_life_time))
   self)
  (:turbulent_diffusion_coefficients
   (&rest __turbulent_diffusion_coefficients)
   (if (keywordp (car __turbulent_diffusion_coefficients))
       (send* _turbulent_diffusion_coefficients __turbulent_diffusion_coefficients)
     (progn
       (if __turbulent_diffusion_coefficients (setq _turbulent_diffusion_coefficients (car __turbulent_diffusion_coefficients)))
       _turbulent_diffusion_coefficients)))
  (:source
   (&rest __source)
   (if (keywordp (car __source))
       (send* _source __source)
     (progn
       (if __source (setq _source (car __source)))
       _source)))
  (:buoyancy_flux
   (&optional __buoyancy_flux)
   (if __buoyancy_flux (setq _buoyancy_flux __buoyancy_flux)) _buoyancy_flux)
  (:stability_param
   (&optional __stability_param)
   (if __stability_param (setq _stability_param __stability_param)) _stability_param)
  (:n_points
   (&optional __n_points)
   (if __n_points (setq _n_points __n_points)) _n_points)
  (:max_particles_per_iter
   (&optional __max_particles_per_iter)
   (if __max_particles_per_iter (setq _max_particles_per_iter __max_particles_per_iter)) _max_particles_per_iter)
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
  (:max_life_time
   (&optional __max_life_time)
   (if __max_life_time (setq _max_life_time __max_life_time)) _max_life_time)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _turbulent_diffusion_coefficients
    (send _turbulent_diffusion_coefficients :serialization-length)
    ;; geometry_msgs/Point _source
    (send _source :serialization-length)
    ;; float64 _buoyancy_flux
    8
    ;; float64 _stability_param
    8
    ;; int32 _n_points
    4
    ;; int32 _max_particles_per_iter
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
    ;; float64 _max_life_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _turbulent_diffusion_coefficients
       (send _turbulent_diffusion_coefficients :serialize s)
     ;; geometry_msgs/Point _source
       (send _source :serialize s)
     ;; float64 _buoyancy_flux
       (sys::poke _buoyancy_flux (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _stability_param
       (sys::poke _stability_param (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _n_points
       (write-long _n_points s)
     ;; int32 _max_particles_per_iter
       (write-long _max_particles_per_iter s)
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
     ;; float64 _max_life_time
       (sys::poke _max_life_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _turbulent_diffusion_coefficients
     (send _turbulent_diffusion_coefficients :deserialize buf ptr-) (incf ptr- (send _turbulent_diffusion_coefficients :serialization-length))
   ;; geometry_msgs/Point _source
     (send _source :deserialize buf ptr-) (incf ptr- (send _source :serialization-length))
   ;; float64 _buoyancy_flux
     (setq _buoyancy_flux (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _stability_param
     (setq _stability_param (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _n_points
     (setq _n_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_particles_per_iter
     (setq _max_particles_per_iter (sys::peek buf ptr- :integer)) (incf ptr- 4)
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
   ;; float64 _max_life_time
     (setq _max_life_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeResponse
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

(defclass uuv_plume_msgs::CreatePassiveScalarTurbulentPlume
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlume :md5sum-) "f374842dc7a898751eeacdffd08129ce")
(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlume :datatype-) "uuv_plume_msgs/CreatePassiveScalarTurbulentPlume")
(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlume :request) uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeRequest)
(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlume :response) uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeResponse)

(defmethod uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeRequest
  (:response () (instance uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeResponse :init)))

(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeRequest :md5sum-) "f374842dc7a898751eeacdffd08129ce")
(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeRequest :datatype-) "uuv_plume_msgs/CreatePassiveScalarTurbulentPlumeRequest")
(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeRequest :definition-)
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

geometry_msgs/Vector3 turbulent_diffusion_coefficients
geometry_msgs/Point source
float64 buoyancy_flux
float64 stability_param
int32 n_points
int32 max_particles_per_iter
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max
float64 max_life_time

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")

(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeResponse :md5sum-) "f374842dc7a898751eeacdffd08129ce")
(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeResponse :datatype-) "uuv_plume_msgs/CreatePassiveScalarTurbulentPlumeResponse")
(setf (get uuv_plume_msgs::CreatePassiveScalarTurbulentPlumeResponse :definition-)
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

geometry_msgs/Vector3 turbulent_diffusion_coefficients
geometry_msgs/Point source
float64 buoyancy_flux
float64 stability_param
int32 n_points
int32 max_particles_per_iter
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max
float64 max_life_time

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")



(provide :uuv_plume_msgs/CreatePassiveScalarTurbulentPlume "f374842dc7a898751eeacdffd08129ce")


