;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::GetNumParticles)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'GetNumParticles (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::GETNUMPARTICLES")
  (make-package "UUV_PLUME_MSGS::GETNUMPARTICLES"))
(unless (find-package "UUV_PLUME_MSGS::GETNUMPARTICLESREQUEST")
  (make-package "UUV_PLUME_MSGS::GETNUMPARTICLESREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::GETNUMPARTICLESRESPONSE")
  (make-package "UUV_PLUME_MSGS::GETNUMPARTICLESRESPONSE"))

(in-package "ROS")





(defclass uuv_plume_msgs::GetNumParticlesRequest
  :super ros::object
  :slots ())

(defmethod uuv_plume_msgs::GetNumParticlesRequest
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

(defclass uuv_plume_msgs::GetNumParticlesResponse
  :super ros::object
  :slots (_num_particles ))

(defmethod uuv_plume_msgs::GetNumParticlesResponse
  (:init
   (&key
    ((:num_particles __num_particles) 0)
    )
   (send-super :init)
   (setq _num_particles (round __num_particles))
   self)
  (:num_particles
   (&optional __num_particles)
   (if __num_particles (setq _num_particles __num_particles)) _num_particles)
  (:serialization-length
   ()
   (+
    ;; int32 _num_particles
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num_particles
       (write-long _num_particles s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num_particles
     (setq _num_particles (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass uuv_plume_msgs::GetNumParticles
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::GetNumParticles :md5sum-) "b3e7a05396001d2e02f04b13547a3da2")
(setf (get uuv_plume_msgs::GetNumParticles :datatype-) "uuv_plume_msgs/GetNumParticles")
(setf (get uuv_plume_msgs::GetNumParticles :request) uuv_plume_msgs::GetNumParticlesRequest)
(setf (get uuv_plume_msgs::GetNumParticles :response) uuv_plume_msgs::GetNumParticlesResponse)

(defmethod uuv_plume_msgs::GetNumParticlesRequest
  (:response () (instance uuv_plume_msgs::GetNumParticlesResponse :init)))

(setf (get uuv_plume_msgs::GetNumParticlesRequest :md5sum-) "b3e7a05396001d2e02f04b13547a3da2")
(setf (get uuv_plume_msgs::GetNumParticlesRequest :datatype-) "uuv_plume_msgs/GetNumParticlesRequest")
(setf (get uuv_plume_msgs::GetNumParticlesRequest :definition-)
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
int32 num_particles

")

(setf (get uuv_plume_msgs::GetNumParticlesResponse :md5sum-) "b3e7a05396001d2e02f04b13547a3da2")
(setf (get uuv_plume_msgs::GetNumParticlesResponse :datatype-) "uuv_plume_msgs/GetNumParticlesResponse")
(setf (get uuv_plume_msgs::GetNumParticlesResponse :definition-)
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
int32 num_particles

")



(provide :uuv_plume_msgs/GetNumParticles "b3e7a05396001d2e02f04b13547a3da2")


