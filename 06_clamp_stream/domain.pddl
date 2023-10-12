(define (domain clamp_transfer)
    (:requirements :negative-preconditions :strips :equality :disjunctive-preconditions)
    ; :derived-predicates
    (:predicates
        ;; There are three beam position states: AtStorage, AtRobot, and AtAssembled. One and Only One is True at the same time.
        (Beam ?beam) ;; Static - List of all beams (beam_id)
        (BeamAtStorage ?beam)
        (BeamAtRobot ?beam)
        (BeamAtAssembled ?beam)

        ;; Joints are defined as the intersection of two beams
        ;; Joints are implied to have order, so (Joint ?beam1 ?beam2) is not the same as (Joint ?beam2 ?beam1)
        ;; ?beam1 have to be assembled before ?beam2 if (Joint ?beam1 ?beam2) is declared
        (Joint ?beam1 ?beam2) ;; Static - List of all joints (beam_id, beam_id)
        
        ;; There are two gripper position states: AtStorage, AtRobot. One and Only One is True at the same time.
        (Gripper ?gripper) ;; Static - List of all grippers (gripper_id)
        (GripperAtRobot ?gripper)
        (GripperAtStorage ?gripper)

        ;; Predicates for gripper type matching
        (BeamNeedsGripperType ?beam ?grippertype) ;; Static - List of all gripper types (can be multiple) required for each beam
        (GripperOfType ?gripper ?grippertype) ;; Static - Statement describing the type of a gripper
               
        ;; Clamps are used to assemble joints between beams
        ;; There are three clamp position states: AtStorage, AtRobot, and AtJoint. One and Only One is True at the same time.
        (Clamp ?clamp) ;; Static - List of clamps (clamp_id)
        (ClampAtStorage ?clamp)
        (ClampAtRobot ?clamp)
        (ClampAtJoint ?clamp ?beam1 ?beam2)

        ;; Predicates for clamp type matching
        (ClampOfType ?clamp ?clamptype) ;; Static
        (JointNeedsClampType ?beam1 ?beam2 ?clamptype) ;; Static

        (AssemblyByClampingMethod ?beam)
        (AssemblyByScrewingMethod ?beam)
        (AssemblyByGroundConnection ?beam)

        ;; Predicates certified by the streams
        (AssembleBeamTraj ?beam ?traj)
        (AssembleBeamNotInCollision ?traj ?heldbeam ?otherbeam)

        ; (ClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
        (AttachClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
        (DetachClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
        (AttachClampTrajNotInCollisionWithClamp ?heldclamp ?beam1 ?beam2 ?traj ?otherclamp ?otherbeam1 ?otherbeam2)
        (DetachClampTrajNotInCollisionWithClamp ?heldclamp ?beam1 ?beam2 ?traj ?otherclamp ?otherbeam1 ?otherbeam2)
        ; (ClampTrajNotInCollisionWithBeam ?traj ?heldclamp ?beam1 ?beam2 ?otherbeam)

        (UnsafeClampTraj ?clamp ?beam1 ?beam2 ?traj)
    )

    (:action assemble_beam_by_clamping_method
        :parameters (?beam ?gripper ?grippertype)
        ; :parameters (?beam ?gripper ?grippertype ?traj)
        :precondition (and
            ;; Beam is at storage
            (BeamAtStorage ?beam)

            ;; Beam Assembly Method is by clamping 
            (AssemblyByClampingMethod ?beam)

            ;; Gripper is already on robot
            (GripperAtRobot ?gripper)

            ;; Gripper type matching:
            (BeamNeedsGripperType ?beam ?grippertype)
            (GripperOfType ?gripper ?grippertype)

            ;; Logic: There should not exist a scenario where an earlierbeam ... 
            (not
                (exists(?earlierbeam)
                    (and 
                        ;; Formed a joint with the current beam ... 
                        (Joint ?earlierbeam ?beam)
                        ;; and that joint is demanding some clamp (declared by JointNeedsClamp) ...
                        (exists(?clamptype) (JointNeedsClampType ?earlierbeam ?beam ?clamptype))
                        ;; and not a single clamp had been attached to that joint
                        (not (exists(?clamp)(ClampAtJoint ?clamp ?earlierbeam ?beam)))
                        ; (NotASingleClampAtJoint ?earlierbeam ?beam)
                    )
                )
            )

            ; (AssembleBeamTraj ?beam ?traj)
            ; (not
            ;   (exists (?otherbeam) (and 
            ;       (BeamAtAssembled ?otherbeam)
            ;       (not (AssembleBeamNotInCollision ?traj ?beam ?otherbeam))
            ;                          ))
            ; )
            )
        :effect (and
            (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
            (BeamAtAssembled ?beam) ;; Beam now at assembled
        )
    )
       
    (:action assemble_beam_by_screwing_method
        :parameters (?beam ?gripper ?grippertype)
        ; :parameters (?beam ?gripper ?grippertype ?traj)
        :precondition (and
            ;; Beam is at storage
            (BeamAtStorage ?beam)

            ;; Beam Assembly Method is by screwing 
            (AssemblyByScrewingMethod ?beam)

            ;; Gripper is already on robot
            (GripperAtRobot ?gripper)

            ;; Gripper type matching:
            (BeamNeedsGripperType ?beam ?grippertype)
            (GripperOfType ?gripper ?grippertype)

            ;; All joints with earlier beams are already assembled
            (not (exists (?earlierbeam)(and
                (Joint ?earlierbeam ?beam)
                (not (BeamAtAssembled ?earlierbeam))
            )))

            ; (AssembleBeamTraj ?beam ?traj)
            ; (not
            ;   (exists (?otherbeam) (and 
            ;       (BeamAtAssembled ?otherbeam)
            ;       (not (AssembleBeamNotInCollision ?traj ?beam ?otherbeam))
            ;                          ))
            ; )
        )
        :effect (and
            (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
            (BeamAtAssembled ?beam) ;; Beam now at assembled
        )
    )

    (:action assemble_beam_by_ground_connection
        :parameters (?beam ?gripper ?grippertype)
        ; :parameters (?beam ?gripper ?grippertype ?traj)
        :precondition (and
            ;; Beam is at storage
            (BeamAtStorage ?beam)

            ;; Beam Assembly Method is by screwing 
            (AssemblyByGroundConnection ?beam)

            ;; Gripper is already on robot
            (GripperAtRobot ?gripper)

            ;; Gripper type matching:
            (BeamNeedsGripperType ?beam ?grippertype)
            (GripperOfType ?gripper ?grippertype)

            ;; All joints with earlier beams are already assembled
            (not (exists (?earlierbeam)(and
                (Joint ?earlierbeam ?beam)
                (not (BeamAtAssembled ?earlierbeam))
            )))

            ; (AssembleBeamTraj ?beam ?traj)
            ; (not
            ;   (exists (?otherbeam) (and 
            ;       (BeamAtAssembled ?otherbeam)
            ;       (not (AssembleBeamNotInCollision ?traj ?beam ?otherbeam))
            ;                          ))
            ; )
        )
        :effect (and
            (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
            (BeamAtAssembled ?beam) ;; Beam now at assembled
        )
    ) 
    ;; Gripper Manipulation
    ;; --------------------
    (:action pick_gripper_from_storage
        :parameters (?gripper ?grippertype)
        :precondition (and
            ;; ?gripper and ?grippertype match at input 
            (GripperOfType ?gripper ?grippertype)
            ;; ?gripper is at storage
            (GripperAtStorage ?gripper)
            ;; Robot is not currently holding a gripper or a clamp
            (not
                (exists
                    (?anytool)
                    (or (GripperAtRobot ?anytool)(ClampAtRobot ?anytool))))
            ;; Robot is not currently holding a beam
            (not
                (exists
                    (?beam)
                    (BeamAtRobot ?beam)))
        )
        :effect (and
            (not (GripperAtStorage ?gripper)) ;; Gripper no longer at storage
            (GripperAtRobot ?gripper) ;; Gripper now at robot
        )
    )

    (:action place_gripper_to_storage
        :parameters (?gripper ?grippertype)
        :precondition (and
            ;; ?gripper and ?grippertype match at input 
            (GripperOfType ?gripper ?grippertype)
            ;; Robot is currently holding ?gripper
            (GripperAtRobot ?gripper)
            ;; Robot is not currently holding a beam
            (not
                (exists
                    (?beam)
                    (BeamAtRobot ?beam)))
        )
        :effect (and
            (not (GripperAtRobot ?gripper)) ;; Gripper no longer at robot
            (GripperAtStorage ?gripper) ;; Gripper now at storage
        )
    )
  
    ; Clamp Manipulation
    ; ------------------

    (:action retrieve_clamp_from_storage
        :parameters (?clamp ?clamptype)
        :precondition (and
            ;; ?clamp and ?clamptype match at input 
            (ClampOfType ?clamp ?clamptype)
            ;; Clamp is at storage
            (ClampAtStorage ?clamp)

            ;; Robot is not currently holding anything
            (not(exists (?anytool)
                    (GripperAtRobot ?anytool)))
            (not(exists (?anytool)
                    (ClampAtRobot ?anytool)))
            (not(exists (?anybeam)
                    (BeamAtRobot ?anybeam)))
        )
        :effect (and
            (not (ClampAtStorage ?clamp)) ;; Gripper no longer at storage
            (ClampAtRobot ?clamp) ;; Gripper now at robot
        )
    )

    (:action store_clamp_to_storage
        :parameters (?clamp ?clamptype)
        :precondition (and
            ;; ?clamp and ?clamptype match at input 
            (ClampOfType ?clamp ?clamptype)
            ;; Robot is currently holding the ?clamp
            (ClampAtRobot ?clamp)
        )
        :effect (and
            (not (ClampAtRobot ?clamp)) ;; Gripper no longer at robot
            (ClampAtStorage ?clamp) ;; Gripper now at storage
        )
    )

    ; this is always False
    ; 
    ; (:derived (UnsafeClampTraj ?clamp ?beam1 ?beam2 ?traj)
    ;     (exists (?otherclamp ?otherbeam1 ?otherbeam2) (and 
    ;         (AttachClampTraj ?clamp ?beam1 ?beam2 ?traj)
    ;         (Joint ?otherbeam1 ?otherbeam2)
    ;         (Clamp ?otherclamp)
    ;         (BeamAtAssembled ?otherbeam1)
    ;         (ClampAtJoint ?otherclamp ?otherbeam1 ?otherbeam2)
    ;         (not (ClampTrajNotInCollisionWithClamp ?clamp ?beam1 ?beam2 ?traj ?otherclamp ?otherbeam1 ?otherbeam2))
    ;         )
    ;     )
    ; )

    (:action attach_clamp_to_structure
        :parameters (?clamp ?clamptype ?beam1 ?beam2 ?traj)
        :precondition (and
            ;; Robot is currently holding the ?clamp
            (ClampAtRobot ?clamp)
            ; The beam where the joint belongs to is already BeamAtAssembled
            (BeamAtAssembled ?beam1)
            (not(BeamAtStorage ?beam1))
            ; The beam where the joint belongs to is already BeamAtAssembled
            (BeamAtStorage ?beam2)
            (not(BeamAtAssembled ?beam2))

            ;; Clamp is suitable for the joint
            (ClampOfType ?clamp ?clamptype)
            (JointNeedsClampType ?beam1 ?beam2 ?clamptype)

            (AttachClampTraj ?clamp ?beam1 ?beam2 ?traj)
            (not
              (exists (?otherclamp ?otherbeam) (and 
                    (ClampAtJoint ?otherclamp ?beam1 ?otherbeam)
                    (Clamp ?otherclamp)
                    (not (= ?otherclamp ?clamp))
                    (not (AttachClampTrajNotInCollisionWithClamp ?clamp ?beam1 ?beam2 ?traj ?otherclamp ?beam1 ?otherbeam))
              ))
            )

            ; (not
            ;   (exists (?otherclamp ?otherbeam1 ?otherbeam2) (and 
            ;         (ClampAtJoint ?otherclamp ?otherbeam1 ?otherbeam2)
            ;         (Clamp ?otherclamp)
            ;         (BeamAtAssembled ?otherbeam1)
            ;         (Joint ?otherbeam1 ?otherbeam2)
            ;         (not (= ?otherclamp ?clamp))
            ;         (not (AttachClampTrajNotInCollisionWithClamp ?heldclamp ?beam1 ?beam2 ?traj ?otherclamp ?otherbeam1 ?otherbeam2))
            ;   ))
            ; )

            ; (not (UnsafeClampTraj ?clamp ?beam1 ?beam2 ?traj))
            ; (ClampTraj ?clamp ?beam1 ?beam2 ?traj)

            ; (not
            ;   (exists (?otherbeam) (and 
            ;       (BeamAtAssembled ?otherbeam)
            ;       (not (ClampTrajNotInCollisionWithBeam ?traj ?clamp ?beam1 ?beam2 ?otherbeam))
            ;                          ))
            ; )
        )
        :effect (and
            (not (ClampAtRobot ?clamp)) ;; Gripper no longer at robot
            (ClampAtJoint ?clamp ?beam1 ?beam2) ;; Gripper now at storage
        )
    )

    (:action detach_clamp_from_structure
        :parameters (?clamp ?clamptype ?beam1 ?beam2 ?traj)
        :precondition (and
            ;; ?clamp and ?clamptype match at input 
            (ClampOfType ?clamp ?clamptype)
            ;; Clamp is at the joint 
            (ClampAtJoint ?clamp ?beam1 ?beam2)

            ;; Robot is not currently holding a gripper or a clamp
            (not(exists (?anytool)
                    (GripperAtRobot ?anytool)))
            (not(exists (?anytool)
                    (ClampAtRobot ?anytool)))

            (DetachClampTraj ?clamp ?beam1 ?beam2 ?traj)
            (not
              (exists (?otherclamp ?otherbeam) (and 
                    (ClampAtJoint ?otherclamp ?beam1 ?otherbeam)
                    (Clamp ?otherclamp)
                    (not (= ?otherclamp ?clamp))
                    (not (DetachClampTrajNotInCollisionWithClamp ?clamp ?beam1 ?beam2 ?traj ?otherclamp ?beam1 ?otherbeam))
              ))
            )

            ; (ClampTraj ?clamp ?beam1 ?beam2 ?traj)
            ; (not
            ;   (exists (?otherclamp ?otherbeam1 ?otherbeam2) (and 
            ;         (Joint ?otherbeam1 ?otherbeam2)
            ;         (ClampAtJoint ?otherclamp ?otherbeam1 ?otherbeam2)
            ;         (not (ClampTrajNotInCollisionWithClamp ?traj ?clamp ?beam1 ?beam2 ?otherclamp))
            ;                          ))
            ; )
            ; (not
            ;   (exists (?otherbeam) (and 
            ;       (BeamAtAssembled ?otherbeam)
            ;       (not (ClampTrajNotInCollisionWithBeam ?traj ?clamp ?beam1 ?beam2 ?otherbeam))
            ;                          ))
            ; )
        )
        :effect (and
            (not (ClampAtJoint ?clamp ?beam1 ?beam2)) ;; Gripper no longer at storage
            (ClampAtRobot ?clamp) ;; Gripper now at robot
        )
    )
)