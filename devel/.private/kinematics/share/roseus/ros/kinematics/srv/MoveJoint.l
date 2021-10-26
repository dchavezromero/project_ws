;; Auto-generated. Do not edit!


(when (boundp 'kinematics::MoveJoint)
  (if (not (find-package "KINEMATICS"))
    (make-package "KINEMATICS"))
  (shadow 'MoveJoint (find-package "KINEMATICS")))
(unless (find-package "KINEMATICS::MOVEJOINT")
  (make-package "KINEMATICS::MOVEJOINT"))
(unless (find-package "KINEMATICS::MOVEJOINTREQUEST")
  (make-package "KINEMATICS::MOVEJOINTREQUEST"))
(unless (find-package "KINEMATICS::MOVEJOINTRESPONSE")
  (make-package "KINEMATICS::MOVEJOINTRESPONSE"))

(in-package "ROS")





(defclass kinematics::MoveJointRequest
  :super ros::object
  :slots (_joint_set_points ))

(defmethod kinematics::MoveJointRequest
  (:init
   (&key
    ((:joint_set_points __joint_set_points) (instance kinematics::joint_angles :init))
    )
   (send-super :init)
   (setq _joint_set_points __joint_set_points)
   self)
  (:joint_set_points
   (&rest __joint_set_points)
   (if (keywordp (car __joint_set_points))
       (send* _joint_set_points __joint_set_points)
     (progn
       (if __joint_set_points (setq _joint_set_points (car __joint_set_points)))
       _joint_set_points)))
  (:serialization-length
   ()
   (+
    ;; kinematics/joint_angles _joint_set_points
    (send _joint_set_points :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kinematics/joint_angles _joint_set_points
       (send _joint_set_points :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kinematics/joint_angles _joint_set_points
     (send _joint_set_points :deserialize buf ptr-) (incf ptr- (send _joint_set_points :serialization-length))
   ;;
   self)
  )

(defclass kinematics::MoveJointResponse
  :super ros::object
  :slots (_valid_position ))

(defmethod kinematics::MoveJointResponse
  (:init
   (&key
    ((:valid_position __valid_position) nil)
    )
   (send-super :init)
   (setq _valid_position __valid_position)
   self)
  (:valid_position
   (&optional __valid_position)
   (if __valid_position (setq _valid_position __valid_position)) _valid_position)
  (:serialization-length
   ()
   (+
    ;; bool _valid_position
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _valid_position
       (if _valid_position (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _valid_position
     (setq _valid_position (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass kinematics::MoveJoint
  :super ros::object
  :slots ())

(setf (get kinematics::MoveJoint :md5sum-) "bc0ca7f92d556a08737d72803818c721")
(setf (get kinematics::MoveJoint :datatype-) "kinematics/MoveJoint")
(setf (get kinematics::MoveJoint :request) kinematics::MoveJointRequest)
(setf (get kinematics::MoveJoint :response) kinematics::MoveJointResponse)

(defmethod kinematics::MoveJointRequest
  (:response () (instance kinematics::MoveJointResponse :init)))

(setf (get kinematics::MoveJointRequest :md5sum-) "bc0ca7f92d556a08737d72803818c721")
(setf (get kinematics::MoveJointRequest :datatype-) "kinematics/MoveJointRequest")
(setf (get kinematics::MoveJointRequest :definition-)
      "joint_angles joint_set_points

================================================================================
MSG: kinematics/joint_angles
float64 theta1
float64 theta2
float64 theta3
float64 theta4
float64 theta5
float64 theta6
---
bool valid_position
")

(setf (get kinematics::MoveJointResponse :md5sum-) "bc0ca7f92d556a08737d72803818c721")
(setf (get kinematics::MoveJointResponse :datatype-) "kinematics/MoveJointResponse")
(setf (get kinematics::MoveJointResponse :definition-)
      "joint_angles joint_set_points

================================================================================
MSG: kinematics/joint_angles
float64 theta1
float64 theta2
float64 theta3
float64 theta4
float64 theta5
float64 theta6
---
bool valid_position
")



(provide :kinematics/MoveJoint "bc0ca7f92d556a08737d72803818c721")


