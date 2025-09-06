; Auto-generated. Do not edit!


(cl:in-package uuv_plume_msgs-srv)


;//! \htmlinclude CreatePassiveScalarTurbulentPlume-request.msg.html

(cl:defclass <CreatePassiveScalarTurbulentPlume-request> (roslisp-msg-protocol:ros-message)
  ((turbulent_diffusion_coefficients
    :reader turbulent_diffusion_coefficients
    :initarg :turbulent_diffusion_coefficients
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (source
    :reader source
    :initarg :source
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (buoyancy_flux
    :reader buoyancy_flux
    :initarg :buoyancy_flux
    :type cl:float
    :initform 0.0)
   (stability_param
    :reader stability_param
    :initarg :stability_param
    :type cl:float
    :initform 0.0)
   (n_points
    :reader n_points
    :initarg :n_points
    :type cl:integer
    :initform 0)
   (max_particles_per_iter
    :reader max_particles_per_iter
    :initarg :max_particles_per_iter
    :type cl:integer
    :initform 0)
   (x_min
    :reader x_min
    :initarg :x_min
    :type cl:float
    :initform 0.0)
   (x_max
    :reader x_max
    :initarg :x_max
    :type cl:float
    :initform 0.0)
   (y_min
    :reader y_min
    :initarg :y_min
    :type cl:float
    :initform 0.0)
   (y_max
    :reader y_max
    :initarg :y_max
    :type cl:float
    :initform 0.0)
   (z_min
    :reader z_min
    :initarg :z_min
    :type cl:float
    :initform 0.0)
   (z_max
    :reader z_max
    :initarg :z_max
    :type cl:float
    :initform 0.0)
   (max_life_time
    :reader max_life_time
    :initarg :max_life_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass CreatePassiveScalarTurbulentPlume-request (<CreatePassiveScalarTurbulentPlume-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreatePassiveScalarTurbulentPlume-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreatePassiveScalarTurbulentPlume-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<CreatePassiveScalarTurbulentPlume-request> is deprecated: use uuv_plume_msgs-srv:CreatePassiveScalarTurbulentPlume-request instead.")))

(cl:ensure-generic-function 'turbulent_diffusion_coefficients-val :lambda-list '(m))
(cl:defmethod turbulent_diffusion_coefficients-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:turbulent_diffusion_coefficients-val is deprecated.  Use uuv_plume_msgs-srv:turbulent_diffusion_coefficients instead.")
  (turbulent_diffusion_coefficients m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:source-val is deprecated.  Use uuv_plume_msgs-srv:source instead.")
  (source m))

(cl:ensure-generic-function 'buoyancy_flux-val :lambda-list '(m))
(cl:defmethod buoyancy_flux-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:buoyancy_flux-val is deprecated.  Use uuv_plume_msgs-srv:buoyancy_flux instead.")
  (buoyancy_flux m))

(cl:ensure-generic-function 'stability_param-val :lambda-list '(m))
(cl:defmethod stability_param-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:stability_param-val is deprecated.  Use uuv_plume_msgs-srv:stability_param instead.")
  (stability_param m))

(cl:ensure-generic-function 'n_points-val :lambda-list '(m))
(cl:defmethod n_points-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:n_points-val is deprecated.  Use uuv_plume_msgs-srv:n_points instead.")
  (n_points m))

(cl:ensure-generic-function 'max_particles_per_iter-val :lambda-list '(m))
(cl:defmethod max_particles_per_iter-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:max_particles_per_iter-val is deprecated.  Use uuv_plume_msgs-srv:max_particles_per_iter instead.")
  (max_particles_per_iter m))

(cl:ensure-generic-function 'x_min-val :lambda-list '(m))
(cl:defmethod x_min-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:x_min-val is deprecated.  Use uuv_plume_msgs-srv:x_min instead.")
  (x_min m))

(cl:ensure-generic-function 'x_max-val :lambda-list '(m))
(cl:defmethod x_max-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:x_max-val is deprecated.  Use uuv_plume_msgs-srv:x_max instead.")
  (x_max m))

(cl:ensure-generic-function 'y_min-val :lambda-list '(m))
(cl:defmethod y_min-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:y_min-val is deprecated.  Use uuv_plume_msgs-srv:y_min instead.")
  (y_min m))

(cl:ensure-generic-function 'y_max-val :lambda-list '(m))
(cl:defmethod y_max-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:y_max-val is deprecated.  Use uuv_plume_msgs-srv:y_max instead.")
  (y_max m))

(cl:ensure-generic-function 'z_min-val :lambda-list '(m))
(cl:defmethod z_min-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:z_min-val is deprecated.  Use uuv_plume_msgs-srv:z_min instead.")
  (z_min m))

(cl:ensure-generic-function 'z_max-val :lambda-list '(m))
(cl:defmethod z_max-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:z_max-val is deprecated.  Use uuv_plume_msgs-srv:z_max instead.")
  (z_max m))

(cl:ensure-generic-function 'max_life_time-val :lambda-list '(m))
(cl:defmethod max_life_time-val ((m <CreatePassiveScalarTurbulentPlume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:max_life_time-val is deprecated.  Use uuv_plume_msgs-srv:max_life_time instead.")
  (max_life_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreatePassiveScalarTurbulentPlume-request>) ostream)
  "Serializes a message object of type '<CreatePassiveScalarTurbulentPlume-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'turbulent_diffusion_coefficients) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'source) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'buoyancy_flux))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'stability_param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'n_points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'max_particles_per_iter)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_life_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreatePassiveScalarTurbulentPlume-request>) istream)
  "Deserializes a message object of type '<CreatePassiveScalarTurbulentPlume-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'turbulent_diffusion_coefficients) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'source) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'buoyancy_flux) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stability_param) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_points) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_particles_per_iter) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_max) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_max) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_max) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_life_time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreatePassiveScalarTurbulentPlume-request>)))
  "Returns string type for a service object of type '<CreatePassiveScalarTurbulentPlume-request>"
  "uuv_plume_msgs/CreatePassiveScalarTurbulentPlumeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreatePassiveScalarTurbulentPlume-request)))
  "Returns string type for a service object of type 'CreatePassiveScalarTurbulentPlume-request"
  "uuv_plume_msgs/CreatePassiveScalarTurbulentPlumeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreatePassiveScalarTurbulentPlume-request>)))
  "Returns md5sum for a message object of type '<CreatePassiveScalarTurbulentPlume-request>"
  "f374842dc7a898751eeacdffd08129ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreatePassiveScalarTurbulentPlume-request)))
  "Returns md5sum for a message object of type 'CreatePassiveScalarTurbulentPlume-request"
  "f374842dc7a898751eeacdffd08129ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreatePassiveScalarTurbulentPlume-request>)))
  "Returns full string definition for message of type '<CreatePassiveScalarTurbulentPlume-request>"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%geometry_msgs/Vector3 turbulent_diffusion_coefficients~%geometry_msgs/Point source~%float64 buoyancy_flux~%float64 stability_param~%int32 n_points~%int32 max_particles_per_iter~%float64 x_min~%float64 x_max~%float64 y_min~%float64 y_max~%float64 z_min~%float64 z_max~%float64 max_life_time~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreatePassiveScalarTurbulentPlume-request)))
  "Returns full string definition for message of type 'CreatePassiveScalarTurbulentPlume-request"
  (cl:format cl:nil "# Copyright (c) 2016 The UUV Simulator Authors.~%# All rights reserved.~%#~%# Licensed under the Apache License, Version 2.0 (the \"License\");~%# you may not use this file except in compliance with the License.~%# You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS,~%# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.~%# See the License for the specific language governing permissions and~%# limitations under the License.~%~%geometry_msgs/Vector3 turbulent_diffusion_coefficients~%geometry_msgs/Point source~%float64 buoyancy_flux~%float64 stability_param~%int32 n_points~%int32 max_particles_per_iter~%float64 x_min~%float64 x_max~%float64 y_min~%float64 y_max~%float64 z_min~%float64 z_max~%float64 max_life_time~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreatePassiveScalarTurbulentPlume-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'turbulent_diffusion_coefficients))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'source))
     8
     8
     4
     4
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreatePassiveScalarTurbulentPlume-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CreatePassiveScalarTurbulentPlume-request
    (cl:cons ':turbulent_diffusion_coefficients (turbulent_diffusion_coefficients msg))
    (cl:cons ':source (source msg))
    (cl:cons ':buoyancy_flux (buoyancy_flux msg))
    (cl:cons ':stability_param (stability_param msg))
    (cl:cons ':n_points (n_points msg))
    (cl:cons ':max_particles_per_iter (max_particles_per_iter msg))
    (cl:cons ':x_min (x_min msg))
    (cl:cons ':x_max (x_max msg))
    (cl:cons ':y_min (y_min msg))
    (cl:cons ':y_max (y_max msg))
    (cl:cons ':z_min (z_min msg))
    (cl:cons ':z_max (z_max msg))
    (cl:cons ':max_life_time (max_life_time msg))
))
;//! \htmlinclude CreatePassiveScalarTurbulentPlume-response.msg.html

(cl:defclass <CreatePassiveScalarTurbulentPlume-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CreatePassiveScalarTurbulentPlume-response (<CreatePassiveScalarTurbulentPlume-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreatePassiveScalarTurbulentPlume-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreatePassiveScalarTurbulentPlume-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uuv_plume_msgs-srv:<CreatePassiveScalarTurbulentPlume-response> is deprecated: use uuv_plume_msgs-srv:CreatePassiveScalarTurbulentPlume-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CreatePassiveScalarTurbulentPlume-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uuv_plume_msgs-srv:success-val is deprecated.  Use uuv_plume_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreatePassiveScalarTurbulentPlume-response>) ostream)
  "Serializes a message object of type '<CreatePassiveScalarTurbulentPlume-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreatePassiveScalarTurbulentPlume-response>) istream)
  "Deserializes a message object of type '<CreatePassiveScalarTurbulentPlume-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreatePassiveScalarTurbulentPlume-response>)))
  "Returns string type for a service object of type '<CreatePassiveScalarTurbulentPlume-response>"
  "uuv_plume_msgs/CreatePassiveScalarTurbulentPlumeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreatePassiveScalarTurbulentPlume-response)))
  "Returns string type for a service object of type 'CreatePassiveScalarTurbulentPlume-response"
  "uuv_plume_msgs/CreatePassiveScalarTurbulentPlumeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreatePassiveScalarTurbulentPlume-response>)))
  "Returns md5sum for a message object of type '<CreatePassiveScalarTurbulentPlume-response>"
  "f374842dc7a898751eeacdffd08129ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreatePassiveScalarTurbulentPlume-response)))
  "Returns md5sum for a message object of type 'CreatePassiveScalarTurbulentPlume-response"
  "f374842dc7a898751eeacdffd08129ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreatePassiveScalarTurbulentPlume-response>)))
  "Returns full string definition for message of type '<CreatePassiveScalarTurbulentPlume-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreatePassiveScalarTurbulentPlume-response)))
  "Returns full string definition for message of type 'CreatePassiveScalarTurbulentPlume-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreatePassiveScalarTurbulentPlume-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreatePassiveScalarTurbulentPlume-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CreatePassiveScalarTurbulentPlume-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CreatePassiveScalarTurbulentPlume)))
  'CreatePassiveScalarTurbulentPlume-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CreatePassiveScalarTurbulentPlume)))
  'CreatePassiveScalarTurbulentPlume-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreatePassiveScalarTurbulentPlume)))
  "Returns string type for a service object of type '<CreatePassiveScalarTurbulentPlume>"
  "uuv_plume_msgs/CreatePassiveScalarTurbulentPlume")