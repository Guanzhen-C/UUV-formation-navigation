;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::DeletePlume)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'DeletePlume (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::DELETEPLUME")
  (make-package "UUV_PLUME_MSGS::DELETEPLUME"))
(unless (find-package "UUV_PLUME_MSGS::DELETEPLUMEREQUEST")
  (make-package "UUV_PLUME_MSGS::DELETEPLUMEREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::DELETEPLUMERESPONSE")
  (make-package "UUV_PLUME_MSGS::DELETEPLUMERESPONSE"))

(in-package "ROS")





(defclass uuv_plume_msgs::DeletePlumeRequest
  :super ros::object
  :slots ())

(defmethod uuv_plume_msgs::DeletePlumeRequest
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

(defclass uuv_plume_msgs::DeletePlumeResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_plume_msgs::DeletePlumeResponse
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

(defclass uuv_plume_msgs::DeletePlume
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::DeletePlume :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_plume_msgs::DeletePlume :datatype-) "uuv_plume_msgs/DeletePlume")
(setf (get uuv_plume_msgs::DeletePlume :request) uuv_plume_msgs::DeletePlumeRequest)
(setf (get uuv_plume_msgs::DeletePlume :response) uuv_plume_msgs::DeletePlumeResponse)

(defmethod uuv_plume_msgs::DeletePlumeRequest
  (:response () (instance uuv_plume_msgs::DeletePlumeResponse :init)))

(setf (get uuv_plume_msgs::DeletePlumeRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_plume_msgs::DeletePlumeRequest :datatype-) "uuv_plume_msgs/DeletePlumeRequest")
(setf (get uuv_plume_msgs::DeletePlumeRequest :definition-)
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
bool success

")

(setf (get uuv_plume_msgs::DeletePlumeResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get uuv_plume_msgs::DeletePlumeResponse :datatype-) "uuv_plume_msgs/DeletePlumeResponse")
(setf (get uuv_plume_msgs::DeletePlumeResponse :definition-)
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
bool success

")



(provide :uuv_plume_msgs/DeletePlume "358e233cde0c8a8bcfea4ce193f8fc15")


