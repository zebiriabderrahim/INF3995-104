; Auto-generated. Do not edit!


(cl:in-package server_communication_handler-srv)


;//! \htmlinclude launch_identify_srv-request.msg.html

(cl:defclass <launch_identify_srv-request> (roslisp-msg-protocol:ros-message)
  ((in
    :reader in
    :initarg :in
    :type cl:string
    :initform ""))
)

(cl:defclass launch_identify_srv-request (<launch_identify_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <launch_identify_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'launch_identify_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name server_communication_handler-srv:<launch_identify_srv-request> is deprecated: use server_communication_handler-srv:launch_identify_srv-request instead.")))

(cl:ensure-generic-function 'in-val :lambda-list '(m))
(cl:defmethod in-val ((m <launch_identify_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader server_communication_handler-srv:in-val is deprecated.  Use server_communication_handler-srv:in instead.")
  (in m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <launch_identify_srv-request>) ostream)
  "Serializes a message object of type '<launch_identify_srv-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'in))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'in))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <launch_identify_srv-request>) istream)
  "Deserializes a message object of type '<launch_identify_srv-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'in) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'in) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<launch_identify_srv-request>)))
  "Returns string type for a service object of type '<launch_identify_srv-request>"
  "server_communication_handler/launch_identify_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'launch_identify_srv-request)))
  "Returns string type for a service object of type 'launch_identify_srv-request"
  "server_communication_handler/launch_identify_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<launch_identify_srv-request>)))
  "Returns md5sum for a message object of type '<launch_identify_srv-request>"
  "2718218ecd3037e7050a0e8416c50c33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'launch_identify_srv-request)))
  "Returns md5sum for a message object of type 'launch_identify_srv-request"
  "2718218ecd3037e7050a0e8416c50c33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<launch_identify_srv-request>)))
  "Returns full string definition for message of type '<launch_identify_srv-request>"
  (cl:format cl:nil "string in~% ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'launch_identify_srv-request)))
  "Returns full string definition for message of type 'launch_identify_srv-request"
  (cl:format cl:nil "string in~% ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <launch_identify_srv-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'in))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <launch_identify_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'launch_identify_srv-request
    (cl:cons ':in (in msg))
))
;//! \htmlinclude launch_identify_srv-response.msg.html

(cl:defclass <launch_identify_srv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass launch_identify_srv-response (<launch_identify_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <launch_identify_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'launch_identify_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name server_communication_handler-srv:<launch_identify_srv-response> is deprecated: use server_communication_handler-srv:launch_identify_srv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <launch_identify_srv-response>) ostream)
  "Serializes a message object of type '<launch_identify_srv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <launch_identify_srv-response>) istream)
  "Deserializes a message object of type '<launch_identify_srv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<launch_identify_srv-response>)))
  "Returns string type for a service object of type '<launch_identify_srv-response>"
  "server_communication_handler/launch_identify_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'launch_identify_srv-response)))
  "Returns string type for a service object of type 'launch_identify_srv-response"
  "server_communication_handler/launch_identify_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<launch_identify_srv-response>)))
  "Returns md5sum for a message object of type '<launch_identify_srv-response>"
  "2718218ecd3037e7050a0e8416c50c33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'launch_identify_srv-response)))
  "Returns md5sum for a message object of type 'launch_identify_srv-response"
  "2718218ecd3037e7050a0e8416c50c33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<launch_identify_srv-response>)))
  "Returns full string definition for message of type '<launch_identify_srv-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'launch_identify_srv-response)))
  "Returns full string definition for message of type 'launch_identify_srv-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <launch_identify_srv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <launch_identify_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'launch_identify_srv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'launch_identify_srv)))
  'launch_identify_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'launch_identify_srv)))
  'launch_identify_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'launch_identify_srv)))
  "Returns string type for a service object of type '<launch_identify_srv>"
  "server_communication_handler/launch_identify_srv")