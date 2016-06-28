
(cl:in-package :asdf)

(defsystem "fake_gps-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Transform" :depends-on ("_package_Transform"))
    (:file "_package_Transform" :depends-on ("_package"))
  ))