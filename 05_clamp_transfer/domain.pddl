(define (domain clamp_transfer)
    (:requirements :negative-preconditions :strips :equality :disjunctive-preconditions)
    ; :derived-predicates
    (:predicates
        ;; There are three beam position states: AtStorage, AtRobot, and AtAssembled. One and Only One is True at the same time.
        (Beam ?beam) ;; Static - List of all beams (beam_id)
        (BeamAtStorage ?beam)
        (BeamAtRobot ?beam)
        (BeamAtAssembled ?beam)
        
        (RobotHasTool)

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

        (JointToolFulfilled ?beam1 ?beam2)

        ;; Predicates for clamp type matching
        (ClampOfType ?clamp ?clamptype) ;; Static
        (JointNeedsClampType ?beam1 ?beam2 ?clamptype) ;; Static

        (AssemblyByClampingMethod ?beam)
        (AssemblyByScrewingMethod ?beam)
        (AssemblyByGroundConnection ?beam)

    )
     (:action assemble_beam_by_clamping_method
        ; :parameters (?beam ?gripper ?grippertype)
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

            ;; All joints with earlier beams are already assembled
            (not (exists (?earlierbeam)(and
                (Joint ?earlierbeam ?beam)
                (not (BeamAtAssembled ?earlierbeam))
            )))

            ;; Logic: There should not exist a scenario where an earlierbeam ... 
            (not
                (exists(?earlierbeam)
                    (and 
                        ;; Formed a joint with the current beam ... 
                        (Joint ?earlierbeam ?beam)
                        ;; and that joint is demanding some clamp (declared by JointNeedsClamp) ...
                        (not (JointToolFulfilled ?earlierbeam ?beam))
                    )
                )
            )
            )
        :effect (and
            (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
            (BeamAtAssembled ?beam) ;; Beam now at assembled
        )
    )
     
    (:action fulfil_joint_tool
        :parameters (?beam1 ?beam2)
        :precondition (and
            (Joint ?beam1 ?beam2)
            (not (JointToolFulfilled ?beam1 ?beam2))
            (not (exists (?clamptype) (JointNeedsClampType ?beam1 ?beam2 ?clamptype)))
        )
        :effect (and
            (JointToolFulfilled ?beam1 ?beam2)
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
        :parameters (?gripper ?grippertype)
        :precondition (and
            ;; ?gripper and ?grippertype match at input 
            (GripperOfType ?gripper ?grippertype)
            ;; ?gripper is at storage
            (GripperAtStorage ?gripper)
            ;; Robot is not currently holding a gripper or a clamp
            (not (RobotHasTool))

        )
        :effect (and
            (not (GripperAtStorage ?gripper)) ;; Gripper no longer at storage
            (GripperAtRobot ?gripper) ;; Gripper now at robot
            (RobotHasTool)

        )
    )

    (:action place_gripper_to_storage
        :parameters (?gripper ?grippertype)
        :precondition (and
            ;; ?gripper and ?grippertype match at input 
            (GripperOfType ?gripper ?grippertype)

            ;; Robot is currently holding ?gripper
            (GripperAtRobot ?gripper)
            (RobotHasTool)

        )
        :effect (and
            (not (GripperAtRobot ?gripper)) ;; Gripper no longer at robot
            (GripperAtStorage ?gripper) ;; Gripper now at storage
            (not (RobotHasTool))

        )
    )
  
    ; Clamp Manipulation
    ; ------------------

    (:action clamp_from_storage_to_joint
        :parameters (?clamp ?clamptype ?beam1 ?beam2)
        :precondition (and
            ;; ?clamp and ?clamptype match at input 
            (ClampOfType ?clamp ?clamptype)
            ;; Clamp is at storage
            (ClampAtStorage ?clamp)

            ; The beam where the joint belongs to is already BeamAtAssembled
            (BeamAtAssembled ?beam1)
            (not(BeamAtStorage ?beam1))
            ; The beam where the joint belongs to is already BeamAtAssembled
            (BeamAtStorage ?beam2)
            (not(BeamAtAssembled ?beam2))

            ;; Clamp is suitable for the joint
            (ClampOfType ?clamp ?clamptype)
            (JointNeedsClampType ?beam1 ?beam2 ?clamptype)
                        
            ;; Robot is not currently holding anything
            (not(RobotHasTool))
        )
        :effect (and
            (not (ClampAtStorage ?clamp)) ;; Gripper no longer at storage
            (ClampAtJoint ?clamp ?beam1 ?beam2) ;; Gripper now at joint
            (JointToolFulfilled ?beam1 ?beam2)
        )
    )

        (:action clamp_from_joint_to_storage
        :parameters (?clamp ?clamptype ?beam1 ?beam2)
        :precondition (and
            
            ;; ?clamp and ?clamptype match at input 
            (Clamp ?clamp)
            (ClampOfType ?clamp ?clamptype)
            ;; Clamp is at the joint 
            (ClampAtJoint ?clamp ?beam1 ?beam2)
            (BeamAtAssembled ?beam1)
            (BeamAtAssembled ?beam2)

            ;; Robot is not currently holding a gripper or a clamp
            (not (RobotHasTool))

        )
        :effect (and
            (not (ClampAtJoint ?clamp ?beam1 ?beam2)) ;; Gripper no longer at robot
            (ClampAtStorage ?clamp) ;; Gripper now at storage
        )
    )

    (:action clamp_from_joint_to_joint
        :parameters (?clamp ?clamptype ?beam_prev_1 ?beam_prev_2 ?beam_next_1 ?beam_next_2)
        :precondition (and
            
            ;; ?clamp and ?clamptype match at input 
            (Clamp ?clamp)
            (ClampOfType ?clamp ?clamptype)
            
            ;; Clamp is at the joint 
            (ClampAtJoint ?clamp ?beam_prev_1 ?beam_prev_2)
            (BeamAtAssembled ?beam_prev_1)
            (BeamAtAssembled ?beam_prev_2)

            ;; Robot is not currently holding a gripper or a clamp
            (not (RobotHasTool))

            ; The beam where the joint belongs to is already BeamAtAssembled
            (BeamAtAssembled ?beam_next_1)
            (not(BeamAtStorage ?beam_next_1))
            ; The beam where the next joint belongs to is not yet assembled
            (BeamAtStorage ?beam_next_2)
            (not(BeamAtAssembled ?beam_next_2))

            ;; Clamp is suitable for the joint
            (ClampOfType ?clamp ?clamptype)
            (JointNeedsClampType ?beam_next_1 ?beam_next_2 ?clamptype)
)
        :effect (and
            (not (ClampAtJoint ?clamp ?beam_prev_1 ?beam_prev_2)) ;; Gripper no longer at robot
            (ClampAtJoint ?clamp ?beam_next_1 ?beam_next_2) ;; Gripper now at storage
            (JointToolFulfilled ?beam_next_1 ?beam_next_2)
        )
    )

)