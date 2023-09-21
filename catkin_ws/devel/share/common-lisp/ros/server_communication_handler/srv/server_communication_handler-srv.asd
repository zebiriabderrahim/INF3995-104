
(cl:in-package :asdf)

(defsystem "server_communication_handler-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "launch_identify_srv" :depends-on ("_package_launch_identify_srv"))
    (:file "_package_launch_identify_srv" :depends-on ("_package"))
  ))