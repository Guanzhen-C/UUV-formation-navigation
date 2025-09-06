
(cl:in-package :asdf)

(defsystem "uuv_plume_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geographic_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ParticleConcentration" :depends-on ("_package_ParticleConcentration"))
    (:file "_package_ParticleConcentration" :depends-on ("_package"))
    (:file "Salinity" :depends-on ("_package_Salinity"))
    (:file "_package_Salinity" :depends-on ("_package"))
  ))