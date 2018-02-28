
(cl:in-package :asdf)

(defsystem "turtlesim_expl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GenValue" :depends-on ("_package_GenValue"))
    (:file "_package_GenValue" :depends-on ("_package"))
  ))