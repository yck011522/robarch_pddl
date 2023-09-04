(define (domain itj_clamp_only)
    (:requirements :negative-preconditions :strips :equality)
    (:predicates
        ; There are thee beam position states: AtStorage, AtRobot, and AtAssembled. One and Only One is True at the same time.
        (Beam ?beam) ;; Static - List of all Beam ID
        (BeamAtStorage ?beam)
        (BeamAtRobot ?beam)
        (BeamAtAssembled ?beam)

        (Gripper ?gripper) ;; Static - List of all Gripper ID
        ; There are two gripper position states: AtStorage, AtRobot. One and Only One is True at the same time.
        (GripperAtRobot ?gripper)
        (GripperAtStorage ?gripper)

        ; Conditions where a beam requires a certain gripper type
        (BeamNeedsGripperType ?beam ?grippertype) ;; Static - List of all gripper types (can be multiple) required for each beam
        (GripperType ?grippertype) ;; Static - List of all gripper types
        (GripperOfType ?gripper ?grippertype) ;; Static - Statement describing the type of a gripper

        ; There are thee clamp position states: AtStorage, AtRobot, and AtJoint. One and Only One is True at the same time.
        (Clamp ?clamp)
        (ClampAtStorage ?clamp)
        (ClampAtRobot ?clamp)
        (ClampAtJoint ?clamp ?beam1 ?beam2)

        ; A Beam requires some clamps at its joints, but it demands a clamp type not a specific clamp id
        (Joint ?beam1 ?beam2)
        (ClampOfType ?clamp ?clamptype) ;; Static
        (BeamHasJoint ?beam ?beam1 ?beam2) ;; Static
        (JointNeedsClamp ?beam1 ?beam2 ?clamptype) ;; Static
        (BeamClampsFulfilled ?beam) ; certified by an action
    )

    (:action pick_beam_with_gripper
        :parameters (?beam ?gripper)
        :precondition (and
            ;; Beam is at storage
            (BeamAtStorage ?beam)
            ;; Gripper is already on robot
            (GripperAtRobot ?gripper)
            ;; Beam requires a certain gripper type
            (exists
                (?grippertype)
                (and
                    (Beam ?beam) ;; (type check)
                    (Gripper ?gripper) ;; (type check)
                    (GripperOfType ?gripper ?grippertype)
                    (BeamNeedsGripperType ?beam ?grippertype))
            )
        )
        :effect (and
            (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
            (BeamAtRobot ?beam) ;; Beam at robot
        )
    )

    (:action assemble_beam
        :parameters (?beam ?gripper)
        :precondition (and
            ;; Gripper is already on robot
            (GripperAtRobot ?gripper)
            ;; Beam is already on robot
            (BeamAtRobot ?beam)
            ;; Beam has all the clamps needed to assemble it
            (BeamClampsFulfilled ?beam)
        )
        :effect (and
            (not (BeamAtRobot ?beam)) ;; Beam no longer at storage
            (BeamAtAssembled ?beam) ;; Beam now at robot
        )
    )

    (:action certify_beam_clamps_fulfilled
        :parameters (?beam)
        :precondition (and
            ;; There should not exist a scenario where an earlierbeam
            (not
                (exists(?earlierbeam)
                    (and 
                        ;; Formed a joint with the current beam
                        (Joint ?earlierbeam ?beam)
                        ;; And that not a single clamp had been attached to that joint
                        (not (exists(?clamp)(ClampAtJoint ?clamp ?earlierbeam ?beam)))
                    )
                )
            )
            (Beam ?beam)
        )
        :effect(and
        (BeamClampsFulfilled ?beam))
    )


    ;; Gripper Manipulation
    ;; --------------------
    (:action pick_gripper_from_storage
        :parameters (?gripper)
        :precondition (and
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
            ;; ?gripper is a valid Gripper
            (GripperAtStorage ?gripper)
            ;; ?gripper is a valid Gripper
            (Gripper ?gripper)
        )
        :effect (and
            (not (GripperAtStorage ?gripper)) ;; Gripper no longer at storage
            (GripperAtRobot ?gripper) ;; Gripper now at robot
        )
    )

    (:action place_gripper_to_storage
        :parameters (?gripper)
        :precondition (and
            ;; ?gripper is a valid Gripper
            (Gripper ?gripper)
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
            (not
                (exists
                    (?anytool)
                    (or (GripperAtRobot ?anytool)(ClampAtRobot ?anytool))))

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
            ; (exists
            ;     (?clamptype)
            ;     (and(ClampOfType ?clamp ?clamptype)(JointNeedsClamp ?beam1 ?beam2) ?clamptype)))
            ;; The beam where the joint belongs to is already BeamAtAssembled
            ; (exists
            ;     (?beam)
            ;     (and (BeamHasJoint ?beam ?beam1 ?beam2)) (BeamAtAssembled ?beam)))
        )
        :effect (and
            (not (ClampAtRobot ?clamp)) ;; Gripper no longer at robot
            (ClampAtJoint ?clamp ?beam1 ?beam2) ;; Gripper now at storage
        )
    )

)