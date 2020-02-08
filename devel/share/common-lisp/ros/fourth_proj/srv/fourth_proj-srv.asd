
(cl:in-package :asdf)

(defsystem "fourth_proj-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "addsrv" :depends-on ("_package_addsrv"))
    (:file "_package_addsrv" :depends-on ("_package"))
  ))