; Auto-generated. Do not edit!


(cl:in-package kinematics-srv)


;//! \htmlinclude MoveJoint-request.msg.html

(cl:defclass <MoveJoint-request> (roslisp-msg-protocol:ros-message)
  ((joint_set_points
    :reader joint_set_points
    :initarg :joint_set_points
    :type kinematics-msg:joint_angles
    :initform (cl:make-instance 'kinematics-msg:joint_angles)))
)

(cl:defclass MoveJoint-request (<MoveJoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveJoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveJoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinematics-srv:<MoveJoint-request> is deprecated: use kinematics-srv:MoveJoint-request instead.")))

(cl:ensure-generic-function 'joint_set_points-val :lambda-list '(m))
(cl:defmethod joint_set_points-val ((m <MoveJoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinematics-srv:joint_set_points-val is deprecated.  Use kinematics-srv:joint_set_points instead.")
  (joint_set_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveJoint-request>) ostream)
  "Serializes a message object of type '<MoveJoint-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_set_points) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveJoint-request>) istream)
  "Deserializes a message object of type '<MoveJoint-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_set_points) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveJoint-request>)))
  "Returns string type for a service object of type '<MoveJoint-request>"
  "kinematics/MoveJointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveJoint-request)))
  "Returns string type for a service object of type 'MoveJoint-request"
  "kinematics/MoveJointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveJoint-request>)))
  "Returns md5sum for a message object of type '<MoveJoint-request>"
  "bc0ca7f92d556a08737d72803818c721")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveJoint-request)))
  "Returns md5sum for a message object of type 'MoveJoint-request"
  "bc0ca7f92d556a08737d72803818c721")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveJoint-request>)))
  "Returns full string definition for message of type '<MoveJoint-request>"
  (cl:format cl:nil "joint_angles joint_set_points~%~%================================================================================~%MSG: kinematics/joint_angles~%float64 theta1~%float64 theta2~%float64 theta3~%float64 theta4~%float64 theta5~%float64 theta6~%float64 theta7~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveJoint-request)))
  "Returns full string definition for message of type 'MoveJoint-request"
  (cl:format cl:nil "joint_angles joint_set_points~%~%================================================================================~%MSG: kinematics/joint_angles~%float64 theta1~%float64 theta2~%float64 theta3~%float64 theta4~%float64 theta5~%float64 theta6~%float64 theta7~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveJoint-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_set_points))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveJoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveJoint-request
    (cl:cons ':joint_set_points (joint_set_points msg))
))
;//! \htmlinclude MoveJoint-response.msg.html

(cl:defclass <MoveJoint-response> (roslisp-msg-protocol:ros-message)
  ((valid_position
    :reader valid_position
    :initarg :valid_position
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveJoint-response (<MoveJoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveJoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveJoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinematics-srv:<MoveJoint-response> is deprecated: use kinematics-srv:MoveJoint-response instead.")))

(cl:ensure-generic-function 'valid_position-val :lambda-list '(m))
(cl:defmethod valid_position-val ((m <MoveJoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinematics-srv:valid_position-val is deprecated.  Use kinematics-srv:valid_position instead.")
  (valid_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveJoint-response>) ostream)
  "Serializes a message object of type '<MoveJoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid_position) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveJoint-response>) istream)
  "Deserializes a message object of type '<MoveJoint-response>"
    (cl:setf (cl:slot-value msg 'valid_position) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveJoint-response>)))
  "Returns string type for a service object of type '<MoveJoint-response>"
  "kinematics/MoveJointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveJoint-response)))
  "Returns string type for a service object of type 'MoveJoint-response"
  "kinematics/MoveJointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveJoint-response>)))
  "Returns md5sum for a message object of type '<MoveJoint-response>"
  "bc0ca7f92d556a08737d72803818c721")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveJoint-response)))
  "Returns md5sum for a message object of type 'MoveJoint-response"
  "bc0ca7f92d556a08737d72803818c721")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveJoint-response>)))
  "Returns full string definition for message of type '<MoveJoint-response>"
  (cl:format cl:nil "bool valid_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveJoint-response)))
  "Returns full string definition for message of type 'MoveJoint-response"
  (cl:format cl:nil "bool valid_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveJoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveJoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveJoint-response
    (cl:cons ':valid_position (valid_position msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveJoint)))
  'MoveJoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveJoint)))
  'MoveJoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveJoint)))
  "Returns string type for a service object of type '<MoveJoint>"
  "kinematics/MoveJoint")