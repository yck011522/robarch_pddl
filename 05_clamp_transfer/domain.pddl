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

        (JointNeedsScrewdriver ?beam1 ?beam2) ;; Static
        (BeamScaffolding ?beam1 ?beam2) ;; Static

        (AssemblyByClampingMethod ?beam)
        (AssemblyByScrewingMethod ?beam)
        (AssemblyByGroundConnection ?beam)

    )
    (:action assemble_beam_by_clamping_method
        :parameters (?beam ?gripper ?grippertype)
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
            
            )
        :effect (and
            (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
            (BeamAtAssembled ?beam) ;; Beam now at assembled
        )
    )
       
    (:action assemble_beam_by_screwing_method
        :parameters (?beam ?gripper ?grippertype)
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
        )
        :effect (and
            (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
            (BeamAtAssembled ?beam) ;; Beam now at assembled
        )
    )

    (:action assemble_beam_by_ground_connection
        :parameters (?beam ?gripper ?grippertype)
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
        )
        :effect (and
            (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
            (BeamAtAssembled ?beam) ;; Beam now at assembled
        )
    ) 
    ;; Gripper Manipulation
    ;; --------------------
    (:action pick_gripper_from_storage
        :parameters (?gripper)
        :precondition (and
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
        :parameters (?gripper)
        :precondition (and
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

    (:action pick_clamp_from_storage
        :parameters (?clamp)
        :precondition (and
            ;; Robot is not currently holding a gripper or a clamp
            (not(exists
                    (?anytool)
                    (or (GripperAtRobot ?anytool)(ClampAtRobot ?anytool))))
            ;; Robot is not currently holding a beam (by magic)
            (not(exists
                    (?anybeam)
                    (BeamAtRobot ?anybeam)))

            ;; Clamp is at storage
            (ClampAtStorage ?clamp)
        )
        :effect (and
            (not (ClampAtStorage ?clamp)) ;; Gripper no longer at storage
            (ClampAtRobot ?clamp) ;; Gripper now at robot
        )
    )

    (:action place_clamp_to_storage
        :parameters (?clamp)
        :precondition (and
            ;; Robot is currently holding the ?clamp
            (ClampAtRobot ?clamp)
        )
        :effect (and
            (not (ClampAtRobot ?clamp)) ;; Gripper no longer at robot
            (ClampAtStorage ?clamp) ;; Gripper now at storage
        )
    )

    (:action pick_clamp_from_joint
        :parameters (?clamp ?beam1 ?beam2)
        :precondition (and
            ;; Robot is not currently holding a gripper or a clamp
            (not
                (exists
                    (?anytool)
                    (or (GripperAtRobot ?anytool)(ClampAtRobot ?anytool))))

            ;; Clamp is a valid clamp
            (ClampAtJoint ?clamp ?beam1 ?beam2)
        )
        :effect (and
            (not (ClampAtJoint ?clamp ?beam1 ?beam2)) ;; Gripper no longer at storage
            (ClampAtRobot ?clamp) ;; Gripper now at robot
        )
    )

    (:action place_clamp_to_joint
        :parameters (?clamp ?beam1 ?beam2)
        :precondition (and
            ;; Robot is currently holding the ?clamp
            (ClampAtRobot ?clamp)
            ;; Clamp is suitable for the joint
            (exists (?clamptype)
                (and(ClampOfType ?clamp ?clamptype)(JointNeedsClampType ?beam1 ?beam2 ?clamptype)))
            ; The beam where the joint belongs to is already BeamAtAssembled
            (BeamAtAssembled ?beam1)
        )
        :effect (and
            (not (ClampAtRobot ?clamp)) ;; Gripper no longer at robot
            (ClampAtJoint ?clamp ?beam1 ?beam2) ;; Gripper now at storage
        )
    )

)