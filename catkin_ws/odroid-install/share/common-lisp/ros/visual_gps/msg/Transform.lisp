; Auto-generated. Do not edit!


(cl:in-package visual_gps-msg)


;//! \htmlinclude Transform.msg.html

(cl:defclass <Transform> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (Transf
    :reader Transf
    :initarg :Transf
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform)))
)

(cl:defclass Transform (<Transform>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Transform>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Transform)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name visual_gps-msg:<Transform> is deprecated: use visual_gps-msg:Transform instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Transform>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_gps-msg:header-val is deprecated.  Use visual_gps-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'Transf-val :lambda-list '(m))
(cl:defmethod Transf-val ((m <Transform>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader visual_gps-msg:Transf-val is deprecated.  Use visual_gps-msg:Transf instead.")
  (Transf m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Transform>) ostream)
  "Serializes a message object of type '<Transform>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Transf) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Transform>) istream)
  "Deserializes a message object of type '<Transform>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Transf) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Transform>)))
  "Returns string type for a message object of type '<Transform>"
  "visual_gps/Transform")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Transform)))
  "Returns string type for a message object of type 'Transform"
  "visual_gps/Transform")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Transform>)))
  "Returns md5sum for a message object of type '<Transform>"
  "915fd0d52ddf8e7b76ed18a14e6bc1ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Transform)))
  "Returns md5sum for a message object of type 'Transform"
  "915fd0d52ddf8e7b76ed18a14e6bc1ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Transform>)))
  "Returns full string definition for message of type '<Transform>"
  (cl:format cl:nil "#Header~%std_msgs/Header header~%~%#Transformation~%geometry_msgs/Transform Transf~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Transform)))
  "Returns full string definition for message of type 'Transform"
  (cl:format cl:nil "#Header~%std_msgs/Header header~%~%#Transformation~%geometry_msgs/Transform Transf~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Transform>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Transf))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Transform>))
  "Converts a ROS message object to a list"
  (cl:list 'Transform
    (cl:cons ':header (header msg))
    (cl:cons ':Transf (Transf msg))
))
