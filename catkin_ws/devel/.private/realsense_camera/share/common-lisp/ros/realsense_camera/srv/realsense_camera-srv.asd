
(cl:in-package :asdf)

(defsystem "realsense_camera-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cameraConfiguration" :depends-on ("_package_cameraConfiguration"))
    (:file "_package_cameraConfiguration" :depends-on ("_package"))
  ))