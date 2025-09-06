;; Auto-generated. Do not edit!


(when (boundp 'uuv_plume_msgs::ParticleConcentration)
  (if (not (find-package "UUV_PLUME_MSGS"))
    (make-package "UUV_PLUME_MSGS"))
  (shadow 'ParticleConcentration (find-package "UUV_PLUME_MSGS")))
(unless (find-package "UUV_PLUME_MSGS::PARTICLECONCENTRATION")
  (make-package "UUV_PLUME_MSGS::PARTICLECONCENTRATION"))

(in-package "ROS")
;;//! \htmlinclude ParticleConcentration.msg.html
(if (not (find-package "GEOGRAPHIC_MSGS"))
  (ros::roseus-add-msgs "geographic_msgs"))
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass uuv_plume_msgs::ParticleConcentration
  :super ros::object
  :slots (_header _position _geo_point _concentration ))

(defmethod uuv_plume_msgs::ParticleConcentration
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:geo_point __geo_point) (instance geographic_msgs::GeoPoint :init))
    ((:concentration __concentration) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _geo_point __geo_point)
   (setq _concentration (float __concentration))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:geo_point
   (&rest __geo_point)
   (if (keywordp (car __geo_point))
       (send* _geo_point __geo_point)
     (progn
       (if __geo_point (setq _geo_point (car __geo_point)))
       _geo_point)))
  (:concentration
   (&optional __concentration)
   (if __concentration (setq _concentration __concentration)) _concentration)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; geographic_msgs/GeoPoint _geo_point
    (send _geo_point :serialization-length)
    ;; float64 _concentration
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; geographic_msgs/GeoPoint _geo_point
       (send _geo_point :serialize s)
     ;; float64 _concentration
       (sys::poke _concentration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geographic_msgs/GeoPoint _geo_point
     (send _geo_point :deserialize buf ptr-) (incf ptr- (send _geo_point :serialization-length))
   ;; float64 _concentration
     (setq _concentration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get uuv_plume_msgs::ParticleConcentration :md5sum-) "018ff018a5cc3c75eb3684e0da416577")
(setf (get uuv_plume_msgs::ParticleConcentration :datatype-) "uuv_plume_msgs/ParticleConcentration")
(setf (get uuv_plume_msgs::ParticleConcentration :definition-)
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

Header header
# Sensor link position wrt the reference frame
geometry_msgs/Vector3 position
# WGS84 coordinates (the altitude component corresponds to -1 * depth)
geographic_msgs/GeoPoint geo_point
# Particle concentration
float64 concentration

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

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
MSG: geographic_msgs/GeoPoint
# Geographic point, using the WGS 84 reference ellipsoid.

# Latitude [degrees]. Positive is north of equator; negative is south
# (-90 <= latitude <= +90).
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is
# west (-180 <= longitude <= +180). At the poles, latitude is -90 or
# +90, and longitude is irrelevant, but must be in range.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
float64 altitude

")



(provide :uuv_plume_msgs/ParticleConcentration "018ff018a5cc3c75eb3684e0da416577")


