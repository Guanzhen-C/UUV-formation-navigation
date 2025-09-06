;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::GetPlumeSourcePosition)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'GetPlumeSourcePosition (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::GETPLUMESOURCEPOSITION")
  (make-package "UUV_PLUME_MSGS::GETPLUMESOURCEPOSITION"))
(unless (find-package "UUV_PLUME_MSGS::GETPLUMESOURCEPOSITIONREQUEST")
  (make-package "UUV_PLUME_MSGS::GETPLUMESOURCEPOSITIONREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::GETPLUMESOURCEPOSITIONRESPONSE")
  (make-package "UUV_PLUME_MSGS::GETPLUMESOURCEPOSITIONRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass uuv_plume_msgs::GetPlumeSourcePositionRequest
  :super ros::object
  :slots ())

(defmethod uuv_plume_msgs::GetPlumeSourcePositionRequest
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

(defclass uuv_plume_msgs::GetPlumeSourcePositionResponse
  :super ros::object
  :slots (_source ))

(defmethod uuv_plume_msgs::GetPlumeSourcePositionResponse
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

(defclass uuv_plume_msgs::GetPlumeSourcePosition
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::GetPlumeSourcePosition :md5sum-) "74e696b4bb2fd623e3aef7e08928baee")
(setf (get uuv_plume_msgs::GetPlumeSourcePosition :datatype-) "uuv_plume_msgs/GetPlumeSourcePosition")
(setf (get uuv_plume_msgs::GetPlumeSourcePosition :request) uuv_plume_msgs::GetPlumeSourcePositionRequest)
(setf (get uuv_plume_msgs::GetPlumeSourcePosition :response) uuv_plume_msgs::GetPlumeSourcePositionResponse)

(defmethod uuv_plume_msgs::GetPlumeSourcePositionRequest
  (:response () (instance uuv_plume_msgs::GetPlumeSourcePositionResponse :init)))

(setf (get uuv_plume_msgs::GetPlumeSourcePositionRequest :md5sum-) "74e696b4bb2fd623e3aef7e08928baee")
(setf (get uuv_plume_msgs::GetPlumeSourcePositionRequest :datatype-) "uuv_plume_msgs/GetPlumeSourcePositionRequest")
(setf (get uuv_plume_msgs::GetPlumeSourcePositionRequest :definition-)
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
geometry_msgs/Point source


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get uuv_plume_msgs::GetPlumeSourcePositionResponse :md5sum-) "74e696b4bb2fd623e3aef7e08928baee")
(setf (get uuv_plume_msgs::GetPlumeSourcePositionResponse :datatype-) "uuv_plume_msgs/GetPlumeSourcePositionResponse")
(setf (get uuv_plume_msgs::GetPlumeSourcePositionResponse :definition-)
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
geometry_msgs/Point source


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :uuv_plume_msgs/GetPlumeSourcePosition "74e696b4bb2fd623e3aef7e08928baee")


