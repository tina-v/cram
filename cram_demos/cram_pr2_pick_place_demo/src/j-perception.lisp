;; load pr2-pick-place-demo
;; load pr2-pms

(roslisp-utilities:startup-ros)

(setf  (btr:joint-state (btr:get-environment-object)
                        "sink_area_dish_washer_door_joint")
       1.4
       (btr:joint-state (btr:get-environment-object)
                         "sink_area_dish_washer_tray_main")
       0.5
       (btr:joint-state (btr:get-environment-object)
                         "sink_area_dish_washer_lower_tray_main")
       0.5)
(btr-belief::publish-environment-joint-state
 (btr:joint-states (btr:get-environment-object)))
(roslisp-utilities:startup-ros)

(pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type parking-arms))))

(pr2-pms:with-real-robot
  (coe:on-event (make-instance 'cpoe:robot-state-changed))
  (exe:perform
   (desig:an action
             (type detecting)
             (object (desig:an object
                               (type j-mug))))))

(pr2-pms:with-real-robot
  (coe:on-event (make-instance 'cpoe:robot-state-changed))
  (exe:perform
   (desig:an action
             (type detecting)
             (object (desig:an object
                               (type j-small-plate))))))

(pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type picking-up)
              (arm left)
              (grasp front)
              (object *))))

(pr2-pms:with-real-robot
   (exe:perform
    (desig:an action
              (type fetching)
              (object *))))



rs::*rs-result*

(btr-utils:kill-all-objects)
(btr:detach-all-objects (btr:get-robot-object))
(giskard::call-giskard-environment-service
           :detach
           :name "jsmallplate_1")
(giskard::call-giskard-environment-service :remove :name "jsmallplate_1")


(pr2-pms:with-real-robot
        (exe:perform
           (desig:an action
                     (type opening-gripper)
                     (gripper left))))

(pr2-pms:with-real-robot
        (exe:perform
           (desig:an action
                     (type closing-gripper)
                     (gripper left))))


(cut:var-value '?obj (car (prolog:prolog `(cpoe:object-in-hand ?obj :left))))


(pr2-pms:with-real-robot
  (let ((?object (cut:var-value '?obj (car (prolog:prolog `(cpoe:object-in-hand ?obj :left)))))
        (?target-pose (cl-transforms-stamped:make-pose-stamped
                       "map" 0.0
                       (cl-transforms:make-3d-vector 1.4 1 0.89)
                       (cl-transforms:make-quaternion 0 0 1 0))))
   (exe:perform
    (desig:an action
              (type placing)
              (object ?object)
              (target (desig:a location 
                               (pose ?target-pose)))
              ))))


(pr2-pms:with-real-robot
           (exe:perform
            (desig:an action
                      (type moving-torso)
                      (joint-angle middle))))   ; upper-limit




(pr2-pms:with-real-robot
  (exe:perform
   (desig:a motion
            (type moving-arm-joints)
            (left-joint-states
             (("l_shoulder_pan_joint" 0.5560452561054948d0)
              ("l_shoulder_lift_joint" -0.36431260705200763d0)
              ("l_upper_arm_roll_joint" 1.0327449989988664d0)
              ("l_elbow_flex_joint" -0.7503198039390824d0)
              ("l_forearm_roll_joint" 38.7912189043761d0)
              ("l_wrist_flex_joint" -1.710500893086413d0)
              ("l_wrist_roll_joint" -78.68244902349872d0))
             ))))


(pr2-pms:with-real-robot
  (exe:perform
   (desig:a motion
            (type moving-arm-joints)
            (left-joint-states
             (("l_shoulder_pan_joint" 0.4007611617958604d0)
              ("l_shoulder_lift_joint" -0.5296956817212164d0)
              ("l_upper_arm_roll_joint" 0.6357064489070282d0)
              ("l_elbow_flex_joint" -1.0010626760553525d0)
              ("l_forearm_roll_joint" 38.555665566313415d0)
              ("l_wrist_flex_joint" -1.710587910926621d0)
              ("l_wrist_roll_joint" -78.68262305917912d0))
             ))))





(defun experiment (?torso-position ?base-pose &optional parking?)
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (giskard::call-giskard-environment-service
   :detach
   :name "jsmallplate_1")
  (giskard::call-giskard-environment-service :remove :name "jsmallplate_1")
  (giskard::call-giskard-environment-service
   :detach
   :name "jbigplate_1")
  (giskard::call-giskard-environment-service :remove :name "jbigplate_1")
  (giskard::call-giskard-environment-service
   :detach
   :name "jmug_1")
  (giskard::call-giskard-environment-service :remove :name "jmug_1")
  (pr2-pms:with-real-robot
    (coe:on-event (make-instance 'cpoe:robot-state-changed))
    (when parking?
      (exe:perform
       (desig:an action
                 (type parking-arms))))
    (exe:perform
     (desig:an action
               (type moving-torso)
               (joint-angle ?torso-position)))
    (let ((?look-pose
            (cl-transforms-stamped:make-pose-stamped
             "base_footprint" 0.0
             (cl-transforms:make-3d-vector
              1.0 0.0 0.6)
             (cl-transforms:make-quaternion
              0.0d0
              0.0d0
              0.19436471674632258d0
              -0.9809293332774394d0))))
      (exe:perform
       (desig:a motion
                (type looking)
                (pose ?look-pose)))
      (exe:perform
       (desig:a motion
                (type looking)
                (pose ?look-pose))))
    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location
                                (pose ?base-pose)))))))
(defun experiment-1 (&optional parking?)
  (experiment :middle
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.3165285250818419d0
                0.7544736542883927d0
                3.573142296445014d-5)
               (cl-transforms:make-quaternion
                3.427743290816828d-5
                -0.001748128231863086d0
                -0.3408198072026266d0
                0.9401270137013972d0))
              parking?))

(defun experiment-2 (&optional parking?)
  (experiment :upper-limit
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.4735557442534411d0 0.8929601057026715d0 -4.313097363934683d-5)
               (cl-transforms:make-quaternion
                3.005799090188459d-4 -0.002049842353459552d0 -0.5620029072817988d0 0.8271326616721467d0))
              parking?))

(defun experiment-3 (&optional parking?)
  (experiment :middle
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.27491482829053504d0 0.6141391509840102d0 -1.0141366975574117d-4)
               (cl-transforms:make-quaternion
                0.001520935710594572d0 -6.15525643967189d-7 -0.19537135642110298d0 0.980728157974657d0))
              parking?))

(defun experiment-4 (&optional parking?)
  (experiment :lower-limit
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.27491482829053504d0 0.6141391509840102d0 -1.0141366975574117d-4)
               (cl-transforms:make-quaternion
                0.001520935710594572d0 -6.15525643967189d-7 -0.19537135642110298d0 0.980728157974657d0))
              parking?))

(defun experiment-5 (&optional parking?)
  (experiment :lower-limit
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.06327933477536551d0 0.12199078479603676d0 2.3063956462038293d-5)
               (cl-transforms:make-quaternion
                -0.001492300358699593d0 0.001611471535727967d0 -0.016091466970502906d0 0.9998681117476775d0))
              parking?))

(defun experiment-6 (&optional parking?)
  (experiment :lower-limit ;:middle
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.4146567071794364d0 -0.4135452315980618d0 -3.109816589861453d-5)
               (cl-transforms:make-quaternion
                -1.6576971749896917d-4 -7.781823789972083d-4 0.35306752623118726d0 0.9355975036700214d0))
              parking?))

(defun experiment-7 (&optional parking?)
  (experiment :middle
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.06327933477536551d0 0.12199078479603676d0 2.3063956462038293d-5)
               (cl-transforms:make-quaternion
                -0.001492300358699593d0 0.001611471535727967d0 -0.016091466970502906d0 0.9998681117476775d0))
              parking?))

(defun experiment-8 (&optional parking?)
  (experiment :lower-limit
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.4735557442534411d0 0.8929601057026715d0 -4.313097363934683d-5)
               (cl-transforms:make-quaternion
                3.005799090188459d-4 -0.002049842353459552d0 -0.5620029072817988d0 0.8271326616721467d0))
              parking?))

(defun experiment-9 (&optional parking?)
  (experiment :lower-limit
              (cl-transforms-stamped:make-pose-stamped
               "map" 0.0
               (cl-transforms:make-3d-vector
                0.394005801497051d0 -0.48263346370033267d0 6.784700712705871d-8)
               (cl-transforms:make-quaternion
                0.0011911611388534912d0 -1.5658899548104312d-4 0.3880497934806542d0 0.9216376263991447d0))
              parking?))


(defun experiment-10 ()
  (pr2-pms:with-real-robot
    (loop for offset from (- (/ pi 2)) to (/ pi 2) by (/ pi 72) do
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pr2-demo random-taba)
                               "Failure happened: ~a~%Skipping..." e)
             (return)))
          (let ((?joint-states
                  `(("l_shoulder_pan_joint" 0.23917616456709512d0)
                    ("l_shoulder_lift_joint" 0.21609316036765588d0)
                    ("l_upper_arm_roll_joint" 0.372403824007675d0)
                    ("l_elbow_flex_joint" -2.19310471358732d0)
                    ("l_forearm_roll_joint" 91.0653174981196d0)
                    ("l_wrist_flex_joint" -1.5099984019074468d0)
                    ("l_wrist_roll_joint" ,(+ -1.2812621586705362d0 offset)))))
            (exe:perform
             (desig:a motion
                      (type moving-arm-joints)
                      (left-joint-states ?joint-states))))
        (coe:on-event (make-instance 'cpoe:robot-state-changed))
        (exe:perform
         (desig:an action
                   (type detecting)
                   (object (desig:an object
                                     (type j-mug)))))))))


(defun experiment-11 ()
  (pr2-pms:with-real-robot
    (loop for offset from (- (/ pi 2)) to (/ pi 2) by (/ pi 72) do
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pr2-demo random-taba)
                               "Failure happened: ~a~%Skipping..." e)
             (return)))
          (let ((?joint-states
                  `(("l_shoulder_pan_joint" 0.1852868632850544d0)
                    ("l_shoulder_lift_joint" 0.44111565582550527d0)
                    ("l_upper_arm_roll_joint" 0.18879754781415525d0)
                    ("l_elbow_flex_joint" -1.9136972406171116d0)
                    ("l_forearm_roll_joint" 89.54746778362384d0)
                    ("l_wrist_flex_joint" -0.7684758766500628d0)
                    ("l_wrist_roll_joint" ,(+ -37.759967439631495d0 offset)))))
            (exe:perform
             (desig:a motion
                      (type moving-arm-joints)
                      (left-joint-states ?joint-states))))
        (coe:on-event (make-instance 'cpoe:robot-state-changed))
        (exe:perform
         (desig:an action
                   (type detecting)
                   (object (desig:an object
                                     (type j-small-plate)))))))))

(defun experiment-12 ()
  (pr2-pms:with-real-robot
    (loop for offset from (- (/ pi 2)) to (/ pi 2) by (/ pi 20) do
      (format t "OFFSET: ~a~%~%~%~%" offset)
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pr2-demo random-taba)
                               "Failure happened: ~a~%Skipping..." e)
             (return)))
          (let ((?joint-states
                  `(("l_shoulder_pan_joint" 0.17376284347243343d0)
                    ("l_shoulder_lift_joint" 0.046818719235642105d0)
                    ("l_upper_arm_roll_joint" 0.7806672119454053d0)
                    ("l_elbow_flex_joint" -1.1105093338612995d0)
                    ("l_forearm_roll_joint" 88.63573984619606d0)
                    ("l_wrist_flex_joint" -0.7660393771244869d0)
                    ("l_wrist_roll_joint" ,(+ -37.0619103255536d0 offset)))))
            (exe:perform
             (desig:a motion
                      (type moving-arm-joints)
                      (left-joint-states ?joint-states))))
        (coe:on-event (make-instance 'cpoe:robot-state-changed))
        (exe:perform
         (desig:an action
                   (type detecting)
                   (object (desig:an object
                                     (type j-small-plate)))))))))

(defun experiment-13 ()
  (pr2-pms:with-real-robot
    (loop for offset from (* 2 (/ pi 20)) to (/ pi 2) by (/ pi 20) do
      (format t "OFFSET: ~a~%~%~%~%" offset)
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pr2-demo random-taba)
                               "Failure happened: ~a~%Skipping..." e)
             (return)))
          (let ((?joint-states
                  `(("l_shoulder_pan_joint" -0.14020451645847165d0)
                    ("l_shoulder_lift_joint" 0.00832802922311519d0)
                    ("l_upper_arm_roll_joint" 0.779705082987186d0)
                    ("l_elbow_flex_joint" -0.9734114701810986d0)
                    ("l_forearm_roll_joint" 87.34407224167698d0)
                    ("l_wrist_flex_joint" -1.3531052360283105d0)
                    ("l_wrist_roll_joint" ,(+ -35.55410869950215d0 offset)))))
            (exe:perform
             (desig:a motion
                      (type moving-arm-joints)
                      (left-joint-states ?joint-states))))
        (coe:on-event (make-instance 'cpoe:robot-state-changed))
        (exe:perform
         (desig:an action
                   (type detecting)
                   (object (desig:an object
                                     (type j-small-plate)))))))))

(defun experiment-14 ()
  (pr2-pms:with-real-robot
    (loop for offset from (* 2 (/ pi 20)) to (/ pi 2) by (/ pi 20) do
      (format t "OFFSET: ~a~%~%~%~%" offset)
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pr2-demo random-taba)
                               "Failure happened: ~a~%Skipping..." e)
             (return)))
          (let ((?joint-states
                  `(("l_shoulder_pan_joint" 0.18246803829491687d0)
                    ("l_shoulder_lift_joint" 0.47486903014418275d0)
                    ("l_upper_arm_roll_joint" 0.37176240470219546d0)
                    ("l_elbow_flex_joint" -0.9440230157702425d0)
                    ("l_forearm_roll_joint" 208.07001715410559d0)
                    ("l_wrist_flex_joint" -1.1455676871532474d0)
                    ("l_wrist_roll_joint" ,(+ -122.23814846368687d0 offset)))))
            (exe:perform
             (desig:a motion
                      (type moving-arm-joints)
                      (left-joint-states ?joint-states))))
        (coe:on-event (make-instance 'cpoe:robot-state-changed))
        (exe:perform
         (desig:an action
                   (type detecting)
                   (object (desig:an object
                                     (type j-mug)))))))))
