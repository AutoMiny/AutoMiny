
(cl:in-package :asdf)

(defsystem "cmvision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Blobs" :depends-on ("_package_Blobs"))
    (:file "_package_Blobs" :depends-on ("_package"))
    (:file "Blob" :depends-on ("_package_Blob"))
    (:file "_package_Blob" :depends-on ("_package"))
  ))