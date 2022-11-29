
(cl:in-package :asdf)

(defsystem "legged_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "walk_msg" :depends-on ("_package_walk_msg"))
    (:file "_package_walk_msg" :depends-on ("_package"))
  ))