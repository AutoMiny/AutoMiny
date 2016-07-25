; Auto-generated. Do not edit!


(cl:in-package realsense_camera-srv)


;//! \htmlinclude cameraConfiguration-request.msg.html

(cl:defclass <cameraConfiguration-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass cameraConfiguration-request (<cameraConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cameraConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cameraConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name realsense_camera-srv:<cameraConfiguration-request> is deprecated: use realsense_camera-srv:cameraConfiguration-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cameraConfiguration-request>) ostream)
  "Serializes a message object of type '<cameraConfiguration-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cameraConfiguration-request>) istream)
  "Deserializes a message object of type '<cameraConfiguration-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cameraConfiguration-request>)))
  "Returns string type for a service object of type '<cameraConfiguration-request>"
  "realsense_camera/cameraConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cameraConfiguration-request)))
  "Returns string type for a service object of type 'cameraConfiguration-request"
  "realsense_camera/cameraConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cameraConfiguration-request>)))
  "Returns md5sum for a message object of type '<cameraConfiguration-request>"
  "2c284890309b239ca006a289ca29b4d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cameraConfiguration-request)))
  "Returns md5sum for a message object of type 'cameraConfiguration-request"
  "2c284890309b239ca006a289ca29b4d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cameraConfiguration-request>)))
  "Returns full string definition for message of type '<cameraConfiguration-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cameraConfiguration-request)))
  "Returns full string definition for message of type 'cameraConfiguration-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cameraConfiguration-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cameraConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cameraConfiguration-request
))
;//! \htmlinclude cameraConfiguration-response.msg.html

(cl:defclass <cameraConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((configuration_str
    :reader configuration_str
    :initarg :configuration_str
    :type cl:string
    :initform ""))
)

(cl:defclass cameraConfiguration-response (<cameraConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cameraConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cameraConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name realsense_camera-srv:<cameraConfiguration-response> is deprecated: use realsense_camera-srv:cameraConfiguration-response instead.")))

(cl:ensure-generic-function 'configuration_str-val :lambda-list '(m))
(cl:defmethod configuration_str-val ((m <cameraConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader realsense_camera-srv:configuration_str-val is deprecated.  Use realsense_camera-srv:configuration_str instead.")
  (configuration_str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cameraConfiguration-response>) ostream)
  "Serializes a message object of type '<cameraConfiguration-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'configuration_str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'configuration_str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cameraConfiguration-response>) istream)
  "Deserializes a message object of type '<cameraConfiguration-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'configuration_str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'configuration_str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cameraConfiguration-response>)))
  "Returns string type for a service object of type '<cameraConfiguration-response>"
  "realsense_camera/cameraConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cameraConfiguration-response)))
  "Returns string type for a service object of type 'cameraConfiguration-response"
  "realsense_camera/cameraConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cameraConfiguration-response>)))
  "Returns md5sum for a message object of type '<cameraConfiguration-response>"
  "2c284890309b239ca006a289ca29b4d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cameraConfiguration-response)))
  "Returns md5sum for a message object of type 'cameraConfiguration-response"
  "2c284890309b239ca006a289ca29b4d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cameraConfiguration-response>)))
  "Returns full string definition for message of type '<cameraConfiguration-response>"
  (cl:format cl:nil "string configuration_str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cameraConfiguration-response)))
  "Returns full string definition for message of type 'cameraConfiguration-response"
  (cl:format cl:nil "string configuration_str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cameraConfiguration-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'configuration_str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cameraConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cameraConfiguration-response
    (cl:cons ':configuration_str (configuration_str msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cameraConfiguration)))
  'cameraConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cameraConfiguration)))
  'cameraConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cameraConfiguration)))
  "Returns string type for a service object of type '<cameraConfiguration>"
  "realsense_camera/cameraConfiguration")