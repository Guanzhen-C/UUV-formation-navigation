;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::LoadPlumeParticles)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'LoadPlumeParticles (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::LOADPLUMEPARTICLES")
  (make-package "UUV_PLUME_MSGS::LOADPLUMEPARTICLES"))
(unless (find-package "UUV_PLUME_MSGS::LOADPLUMEPARTICLESREQUEST")
  (make-package "UUV_PLUME_MSGS::LOADPLUMEPARTICLESREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::LOADPLUMEPARTICLESRESPONSE")
  (make-package "UUV_PLUME_MSGS::LOADPLUMEPARTICLESRESPONSE"))

(in-package "ROS")





(defclass uuv_plume_msgs::LoadPlumeParticlesRequest
  :super ros::object
  :slots (_plume_file ))

(defmethod uuv_plume_msgs::LoadPlumeParticlesRequest
  (:init
   (&key
    ((:plume_file __plume_file) "")
    )
   (send-super :init)
   (setq _plume_file (string __plume_file))
   self)
  (:plume_file
   (&optional __plume_file)
   (if __plume_file (setq _plume_file __plume_file)) _plume_file)
  (:serialization-length
   ()
   (+
    ;; string _plume_file
    4 (length _plume_file)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _plume_file
       (write-long (length _plume_file) s) (princ _plume_file s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _plume_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _plume_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uuv_plume_msgs::LoadPlumeParticlesResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_plume_msgs::LoadPlumeParticlesResponse
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

(defclass uuv_plume_msgs::LoadPlumeParticles
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::LoadPlumeParticles :md5sum-) "89cea82104ad560d0ae7df2662f9b983")
(setf (get uuv_plume_msgs::LoadPlumeParticles :datatype-) "uuv_plume_msgs/LoadPlumeParticles")
(setf (get uuv_plume_msgs::LoadPlumeParticles :request) uuv_plume_msgs::LoadPlumeParticlesRequest)
(setf (get uuv_plume_msgs::LoadPlumeParticles :response) uuv_plume_msgs::LoadPlumeParticlesResponse)

(defmethod uuv_plume_msgs::LoadPlumeParticlesRequest
  (:response () (instance uuv_plume_msgs::LoadPlumeParticlesResponse :init)))

(setf (get uuv_plume_msgs::LoadPlumeParticlesRequest :md5sum-) "89cea82104ad560d0ae7df2662f9b983")
(setf (get uuv_plume_msgs::LoadPlumeParticlesRequest :datatype-) "uuv_plume_msgs/LoadPlumeParticlesRequest")
(setf (get uuv_plume_msgs::LoadPlumeParticlesRequest :definition-)
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

string plume_file
---
bool success
")

(setf (get uuv_plume_msgs::LoadPlumeParticlesResponse :md5sum-) "89cea82104ad560d0ae7df2662f9b983")
(setf (get uuv_plume_msgs::LoadPlumeParticlesResponse :datatype-) "uuv_plume_msgs/LoadPlumeParticlesResponse")
(setf (get uuv_plume_msgs::LoadPlumeParticlesResponse :definition-)
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

string plume_file
---
bool success
")



(provide :uuv_plume_msgs/LoadPlumeParticles "89cea82104ad560d0ae7df2662f9b983")


