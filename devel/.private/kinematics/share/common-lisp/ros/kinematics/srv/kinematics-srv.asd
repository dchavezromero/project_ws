
(cl:in-package :asdf)

(defsystem "kinematics-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :kinematics-msg
)
  :components ((:file "_package")
    (:file "MoveJoint" :depends-on ("_package_MoveJoint"))
    (:file "_package_MoveJoint" :depends-on ("_package"))
  ))