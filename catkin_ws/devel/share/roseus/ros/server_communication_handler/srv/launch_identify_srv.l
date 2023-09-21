;; Auto-generated. Do not edit!


(when (boundp 'server_communication_handler::launch_identify_srv)
  (if (not (find-package "SERVER_COMMUNICATION_HANDLER"))
    (make-package "SERVER_COMMUNICATION_HANDLER"))
  (shadow 'launch_identify_srv (find-package "SERVER_COMMUNICATION_HANDLER")))
(unless (find-package "SERVER_COMMUNICATION_HANDLER::LAUNCH_IDENTIFY_SRV")
  (make-package "SERVER_COMMUNICATION_HANDLER::LAUNCH_IDENTIFY_SRV"))
(unless (find-package "SERVER_COMMUNICATION_HANDLER::LAUNCH_IDENTIFY_SRVREQUEST")
  (make-package "SERVER_COMMUNICATION_HANDLER::LAUNCH_IDENTIFY_SRVREQUEST"))
(unless (find-package "SERVER_COMMUNICATION_HANDLER::LAUNCH_IDENTIFY_SRVRESPONSE")
  (make-package "SERVER_COMMUNICATION_HANDLER::LAUNCH_IDENTIFY_SRVRESPONSE"))

(in-package "ROS")





(defclass server_communication_handler::launch_identify_srvRequest
  :super ros::object
  :slots (_in ))

(defmethod server_communication_handler::launch_identify_srvRequest
  (:init
   (&key
    ((:in __in) "")
    )
   (send-super :init)
   (setq _in (string __in))
   self)
  (:in
   (&optional __in)
   (if __in (setq _in __in)) _in)
  (:serialization-length
   ()
   (+
    ;; string _in
    4 (length _in)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _in
       (write-long (length _in) s) (princ _in s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _in
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _in (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass server_communication_handler::launch_identify_srvResponse
  :super ros::object
  :slots ())

(defmethod server_communication_handler::launch_identify_srvResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass server_communication_handler::launch_identify_srv
  :super ros::object
  :slots ())

(setf (get server_communication_handler::launch_identify_srv :md5sum-) "2718218ecd3037e7050a0e8416c50c33")
(setf (get server_communication_handler::launch_identify_srv :datatype-) "server_communication_handler/launch_identify_srv")
(setf (get server_communication_handler::launch_identify_srv :request) server_communication_handler::launch_identify_srvRequest)
(setf (get server_communication_handler::launch_identify_srv :response) server_communication_handler::launch_identify_srvResponse)

(defmethod server_communication_handler::launch_identify_srvRequest
  (:response () (instance server_communication_handler::launch_identify_srvResponse :init)))

(setf (get server_communication_handler::launch_identify_srvRequest :md5sum-) "2718218ecd3037e7050a0e8416c50c33")
(setf (get server_communication_handler::launch_identify_srvRequest :datatype-) "server_communication_handler/launch_identify_srvRequest")
(setf (get server_communication_handler::launch_identify_srvRequest :definition-)
      "string in
 
---


")

(setf (get server_communication_handler::launch_identify_srvResponse :md5sum-) "2718218ecd3037e7050a0e8416c50c33")
(setf (get server_communication_handler::launch_identify_srvResponse :datatype-) "server_communication_handler/launch_identify_srvResponse")
(setf (get server_communication_handler::launch_identify_srvResponse :definition-)
      "string in
 
---


")



(provide :server_communication_handler/launch_identify_srv "2718218ecd3037e7050a0e8416c50c33")


