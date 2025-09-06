;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::StorePlumeState)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'StorePlumeState (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::STOREPLUMESTATE")
  (make-package "UUV_PLUME_MSGS::STOREPLUMESTATE"))
(unless (find-package "UUV_PLUME_MSGS::STOREPLUMESTATEREQUEST")
  (make-package "UUV_PLUME_MSGS::STOREPLUMESTATEREQUEST"))
(unless (find-package "UUV_PLUME_MSGS::STOREPLUMESTATERESPONSE")
  (make-package "UUV_PLUME_MSGS::STOREPLUMESTATERESPONSE"))

(in-package "ROS")





(defclass uuv_plume_msgs::StorePlumeStateRequest
  :super ros::object
  :slots (_output_dir _filename ))

(defmethod uuv_plume_msgs::StorePlumeStateRequest
  (:init
   (&key
    ((:output_dir __output_dir) "")
    ((:filename __filename) "")
    )
   (send-super :init)
   (setq _output_dir (string __output_dir))
   (setq _filename (string __filename))
   self)
  (:output_dir
   (&optional __output_dir)
   (if __output_dir (setq _output_dir __output_dir)) _output_dir)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:serialization-length
   ()
   (+
    ;; string _output_dir
    4 (length _output_dir)
    ;; string _filename
    4 (length _filename)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _output_dir
       (write-long (length _output_dir) s) (princ _output_dir s)
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _output_dir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _output_dir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uuv_plume_msgs::StorePlumeStateResponse
  :super ros::object
  :slots (_success _plume_file ))

(defmethod uuv_plume_msgs::StorePlumeStateResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:plume_file __plume_file) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _plume_file (string __plume_file))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:plume_file
   (&optional __plume_file)
   (if __plume_file (setq _plume_file __plume_file)) _plume_file)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _plume_file
    4 (length _plume_file)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _plume_file
       (write-long (length _plume_file) s) (princ _plume_file s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _plume_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _plume_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uuv_plume_msgs::StorePlumeState
  :super ros::object
  :slots ())

(setf (get uuv_plume_msgs::StorePlumeState :md5sum-) "4fbc4c340238072a838f039245e40b09")
(setf (get uuv_plume_msgs::StorePlumeState :datatype-) "uuv_plume_msgs/StorePlumeState")
(setf (get uuv_plume_msgs::StorePlumeState :request) uuv_plume_msgs::StorePlumeStateRequest)
(setf (get uuv_plume_msgs::StorePlumeState :response) uuv_plume_msgs::StorePlumeStateResponse)

(defmethod uuv_plume_msgs::StorePlumeStateRequest
  (:response () (instance uuv_plume_msgs::StorePlumeStateResponse :init)))

(setf (get uuv_plume_msgs::StorePlumeStateRequest :md5sum-) "4fbc4c340238072a838f039245e40b09")
(setf (get uuv_plume_msgs::StorePlumeStateRequest :datatype-) "uuv_plume_msgs/StorePlumeStateRequest")
(setf (get uuv_plume_msgs::StorePlumeStateRequest :definition-)
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

string output_dir
string filename
---
bool success
string plume_file
")

(setf (get uuv_plume_msgs::StorePlumeStateResponse :md5sum-) "4fbc4c340238072a838f039245e40b09")
(setf (get uuv_plume_msgs::StorePlumeStateResponse :datatype-) "uuv_plume_msgs/StorePlumeStateResponse")
(setf (get uuv_plume_msgs::StorePlumeStateResponse :definition-)
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

string output_dir
string filename
---
bool success
string plume_file
")



(provide :uuv_plume_msgs/StorePlumeState "4fbc4c340238072a838f039245e40b09")


