;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::SetPlumeConfiguration)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'SetPlumeConfiguration (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMECONFIGURATION")
  (make-package "UUV_PLUME_MSGS::SETPLUMECONFIGURATION"))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMECONFIGURATIONREQUEST")
  (make-package "UUV_PLUME_MSGS::SETPLUMECONFIGURATIONREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMECONFIGURATIONRESPONSE")
  (make-package "UUV_PLUME_MSGS::SETPLUMECONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass uuv_plume_msgs::SetPlumeConfigurationRequest
  :super ros::object
  :slots (_n_points _max_particles_per_iter ))

(defmethod uuv_plume_msgs::SetPlumeConfigurationRequest
  (:init
   (&key
    ((:n_points __n_points) 0)
    ((:max_particles_per_iter __max_particles_per_iter) 0)
    )
   (send-super :init)
   (setq _n_points (round __n_points))
   (setq _max_particles_per_iter (round __max_particles_per_iter))
   self)
  (:n_points
   (&optional __n_points)
   (if __n_points (setq _n_points __n_points)) _n_points)
  (:max_particles_per_iter
   (&optional __max_particles_per_iter)
   (if __max_particles_per_iter (setq _max_particles_per_iter __max_particles_per_iter)) _max_particles_per_iter)
  (:serialization-length
   ()
   (+
    ;; int32 _n_points
    4
    ;; int32 _max_particles_per_iter
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _n_points
       (write-long _n_points s)
     ;; int32 _max_particles_per_iter
       (write-long _max_particles_per_iter s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _n_points
     (setq _n_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_particles_per_iter
     (setq _max_particles_per_iter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass uuv_plume_msgs::SetPlumeConfigurationResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_plume_msgs::SetPlumeConfigurationResponse
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

(defclass uuv_plume_msgs::SetPlumeConfiguration
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::SetPlumeConfiguration :md5sum-) "55045be54c5c1ddb03ae4cfc1750c6aa")
(setf (get uuv_plume_msgs::SetPlumeConfiguration :datatype-) "uuv_plume_msgs/SetPlumeConfiguration")
(setf (get uuv_plume_msgs::SetPlumeConfiguration :request) uuv_plume_msgs::SetPlumeConfigurationRequest)
(setf (get uuv_plume_msgs::SetPlumeConfiguration :response) uuv_plume_msgs::SetPlumeConfigurationResponse)

(defmethod uuv_plume_msgs::SetPlumeConfigurationRequest
  (:response () (instance uuv_plume_msgs::SetPlumeConfigurationResponse :init)))

(setf (get uuv_plume_msgs::SetPlumeConfigurationRequest :md5sum-) "55045be54c5c1ddb03ae4cfc1750c6aa")
(setf (get uuv_plume_msgs::SetPlumeConfigurationRequest :datatype-) "uuv_plume_msgs/SetPlumeConfigurationRequest")
(setf (get uuv_plume_msgs::SetPlumeConfigurationRequest :definition-)
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

int32 n_points
int32 max_particles_per_iter
---
bool success

")

(setf (get uuv_plume_msgs::SetPlumeConfigurationResponse :md5sum-) "55045be54c5c1ddb03ae4cfc1750c6aa")
(setf (get uuv_plume_msgs::SetPlumeConfigurationResponse :datatype-) "uuv_plume_msgs/SetPlumeConfigurationResponse")
(setf (get uuv_plume_msgs::SetPlumeConfigurationResponse :definition-)
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

int32 n_points
int32 max_particles_per_iter
---
bool success

")



(provide :uuv_plume_msgs/SetPlumeConfiguration "55045be54c5c1ddb03ae4cfc1750c6aa")


