;; Auto-generated. Do not edit!


(when (boundp 'kinematics::joint_angles)
  (if (not (find-package "KINEMATICS"))
    (make-package "KINEMATICS"))
  (shadow 'joint_angles (find-package "KINEMATICS")))
(unless (find-package "KINEMATICS::JOINT_ANGLES")
  (make-package "KINEMATICS::JOINT_ANGLES"))

(in-package "ROS")
;;//! \htmlinclude joint_angles.msg.html


(defclass kinematics::joint_angles
  :super ros::object
  :slots (_theta1 _theta2 _theta3 _theta4 _theta5 _theta6 _theta7 ))

(defmethod kinematics::joint_angles
  (:init
   (&key
    ((:theta1 __theta1) 0.0)
    ((:theta2 __theta2) 0.0)
    ((:theta3 __theta3) 0.0)
    ((:theta4 __theta4) 0.0)
    ((:theta5 __theta5) 0.0)
    ((:theta6 __theta6) 0.0)
    ((:theta7 __theta7) 0.0)
    )
   (send-super :init)
   (setq _theta1 (float __theta1))
   (setq _theta2 (float __theta2))
   (setq _theta3 (float __theta3))
   (setq _theta4 (float __theta4))
   (setq _theta5 (float __theta5))
   (setq _theta6 (float __theta6))
   (setq _theta7 (float __theta7))
   self)
  (:theta1
   (&optional __theta1)
   (if __theta1 (setq _theta1 __theta1)) _theta1)
  (:theta2
   (&optional __theta2)
   (if __theta2 (setq _theta2 __theta2)) _theta2)
  (:theta3
   (&optional __theta3)
   (if __theta3 (setq _theta3 __theta3)) _theta3)
  (:theta4
   (&optional __theta4)
   (if __theta4 (setq _theta4 __theta4)) _theta4)
  (:theta5
   (&optional __theta5)
   (if __theta5 (setq _theta5 __theta5)) _theta5)
  (:theta6
   (&optional __theta6)
   (if __theta6 (setq _theta6 __theta6)) _theta6)
  (:theta7
   (&optional __theta7)
   (if __theta7 (setq _theta7 __theta7)) _theta7)
  (:serialization-length
   ()
   (+
    ;; float64 _theta1
    8
    ;; float64 _theta2
    8
    ;; float64 _theta3
    8
    ;; float64 _theta4
    8
    ;; float64 _theta5
    8
    ;; float64 _theta6
    8
    ;; float64 _theta7
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _theta1
       (sys::poke _theta1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta2
       (sys::poke _theta2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta3
       (sys::poke _theta3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta4
       (sys::poke _theta4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta5
       (sys::poke _theta5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta6
       (sys::poke _theta6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta7
       (sys::poke _theta7 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _theta1
     (setq _theta1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta2
     (setq _theta2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta3
     (setq _theta3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta4
     (setq _theta4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta5
     (setq _theta5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta6
     (setq _theta6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta7
     (setq _theta7 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get kinematics::joint_angles :md5sum-) "b8d6cfc93d9c18a43f67f9435ffa4b9a")
(setf (get kinematics::joint_angles :datatype-) "kinematics/joint_angles")
(setf (get kinematics::joint_angles :definition-)
      "float64 theta1
float64 theta2
float64 theta3
float64 theta4
float64 theta5
float64 theta6
float64 theta7
")



(provide :kinematics/joint_angles "b8d6cfc93d9c18a43f67f9435ffa4b9a")


