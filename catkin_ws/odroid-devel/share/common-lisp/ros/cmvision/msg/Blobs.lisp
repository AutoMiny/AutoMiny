; Auto-generated. Do not edit!


(cl:in-package cmvision-msg)


;//! \htmlinclude Blobs.msg.html

(cl:defclass <Blobs> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image_width
    :reader image_width
    :initarg :image_width
    :type cl:integer
    :initform 0)
   (image_height
    :reader image_height
    :initarg :image_height
    :type cl:integer
    :initform 0)
   (blob_count
    :reader blob_count
    :initarg :blob_count
    :type cl:integer
    :initform 0)
   (blobs
    :reader blobs
    :initarg :blobs
    :type (cl:vector cmvision-msg:Blob)
   :initform (cl:make-array 0 :element-type 'cmvision-msg:Blob :initial-element (cl:make-instance 'cmvision-msg:Blob))))
)

(cl:defclass Blobs (<Blobs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Blobs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Blobs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cmvision-msg:<Blobs> is deprecated: use cmvision-msg:Blobs instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Blobs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmvision-msg:header-val is deprecated.  Use cmvision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <Blobs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmvision-msg:image_width-val is deprecated.  Use cmvision-msg:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <Blobs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmvision-msg:image_height-val is deprecated.  Use cmvision-msg:image_height instead.")
  (image_height m))

(cl:ensure-generic-function 'blob_count-val :lambda-list '(m))
(cl:defmethod blob_count-val ((m <Blobs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmvision-msg:blob_count-val is deprecated.  Use cmvision-msg:blob_count instead.")
  (blob_count m))

(cl:ensure-generic-function 'blobs-val :lambda-list '(m))
(cl:defmethod blobs-val ((m <Blobs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cmvision-msg:blobs-val is deprecated.  Use cmvision-msg:blobs instead.")
  (blobs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Blobs>) ostream)
  "Serializes a message object of type '<Blobs>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'blob_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'blob_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'blob_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'blob_count)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'blobs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'blobs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Blobs>) istream)
  "Deserializes a message object of type '<Blobs>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'blob_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'blob_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'blob_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'blob_count)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'blobs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'blobs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cmvision-msg:Blob))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Blobs>)))
  "Returns string type for a message object of type '<Blobs>"
  "cmvision/Blobs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Blobs)))
  "Returns string type for a message object of type 'Blobs"
  "cmvision/Blobs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Blobs>)))
  "Returns md5sum for a message object of type '<Blobs>"
  "9095431d60142fc813f87d8cc9018af4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Blobs)))
  "Returns md5sum for a message object of type 'Blobs"
  "9095431d60142fc813f87d8cc9018af4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Blobs>)))
  "Returns full string definition for message of type '<Blobs>"
  (cl:format cl:nil "Header header~%uint32 image_width~%uint32 image_height~%uint32 blob_count~%Blob[] blobs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cmvision/Blob~%string name~%uint32 red~%uint32 green~%uint32 blue~%uint32 area~%uint32 x~%uint32 y~%uint32 left~%uint32 right~%uint32 top~%uint32 bottom~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Blobs)))
  "Returns full string definition for message of type 'Blobs"
  (cl:format cl:nil "Header header~%uint32 image_width~%uint32 image_height~%uint32 blob_count~%Blob[] blobs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cmvision/Blob~%string name~%uint32 red~%uint32 green~%uint32 blue~%uint32 area~%uint32 x~%uint32 y~%uint32 left~%uint32 right~%uint32 top~%uint32 bottom~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Blobs>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blobs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Blobs>))
  "Converts a ROS message object to a list"
  (cl:list 'Blobs
    (cl:cons ':header (header msg))
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
    (cl:cons ':blob_count (blob_count msg))
    (cl:cons ':blobs (blobs msg))
))
