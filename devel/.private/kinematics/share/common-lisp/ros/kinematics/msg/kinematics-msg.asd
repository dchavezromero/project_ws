
(cl:in-package :asdf)

(defsystem "kinematics-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "joint_angles" :depends-on ("_package_joint_angles"))
    (:file "_package_joint_angles" :depends-on ("_package"))
  ))