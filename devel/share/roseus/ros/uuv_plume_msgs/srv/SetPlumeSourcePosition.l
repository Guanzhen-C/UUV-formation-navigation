;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::SetPlumeSourcePosition)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'SetPlumeSourcePosition (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMESOURCEPOSITION")
  (make-package "UUV_PLUME_MSGS::SETPLUMESOURCEPOSITION"))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMESOURCEPOSITIONREQUEST")
  (make-package "UUV_PLUME_MSGS::SETPLUMESOURCEPOSITIONREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::SETPLUMESOURCEPOSITIONRESPONSE")
  (make-package "UUV_PLUME_MSGS::SETPLUMESOURCEPOSITIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass uuv_plume_msgs::SetPlumeSourcePositionRequest
  :super ros::object
  :slots (_source ))

(defmethod uuv_plume_msgs::SetPlumeSourcePositionRequest
  (:init
   (&key
    ((:source __source) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _source __source)
   self)
  (:source
   (&rest __source)
   (if (keywordp (car __source))
       (send* _source __source)
     (progn
       (if __source (setq _source (car __source)))
       _source)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _source
    (send _source :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _source
       (send _source :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _source
     (send _source :deserialize buf ptr-) (incf ptr- (send _source :serialization-length))
   ;;
   self)
  )

(defclass uuv_plume_msgs::SetPlumeSourcePositionResponse
  :super ros::object
  :slots (_success ))

(defmethod uuv_plume_msgs::SetPlumeSourcePositionResponse
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

(defclass uuv_plume_msgs::SetPlumeSourcePosition
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::SetPlumeSourcePosition :md5sum-) "6f914c566d493481f609db28b34106cb")
(setf (get uuv_plume_msgs::SetPlumeSourcePosition :datatype-) "uuv_plume_msgs/SetPlumeSourcePosition")
(setf (get uuv_plume_msgs::SetPlumeSourcePosition :request) uuv_plume_msgs::SetPlumeSourcePositionRequest)
(setf (get uuv_plume_msgs::SetPlumeSourcePosition :response) uuv_plume_msgs::SetPlumeSourcePositionResponse)

(defmethod uuv_plume_msgs::SetPlumeSourcePositionRequest
  (:response () (instance uuv_plume_msgs::SetPlumeSourcePositionResponse :init)))

(setf (get uuv_plume_msgs::SetPlumeSourcePositionRequest :md5sum-) "6f914c566d493481f609db28b34106cb")
(setf (get uuv_plume_msgs::SetPlumeSourcePositionRequest :datatype-) "uuv_plume_msgs/SetPlumeSourcePositionRequest")
(setf (get uuv_plume_msgs::SetPlumeSourcePositionRequest :definition-)
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

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")

(setf (get uuv_plume_msgs::SetPlumeSourcePositionResponse :md5sum-) "6f914c566d493481f609db28b34106cb")
(setf (get uuv_plume_msgs::SetPlumeSourcePositionResponse :datatype-) "uuv_plume_msgs/SetPlumeSourcePositionResponse")
(setf (get uuv_plume_msgs::SetPlumeSourcePositionResponse :definition-)
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

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")



(provide :uuv_plume_msgs/SetPlumeSourcePosition "6f914c566d493481f609db28b34106cb")


