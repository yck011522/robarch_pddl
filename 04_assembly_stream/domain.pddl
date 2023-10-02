(define (domain assembly_stream)
  (:requirements :negative-preconditions :strips :equality)
  (:predicates
    ; There are thee beam position states: AtStorage, AtRobot, and AtAssembled. One and Only One is True at the same time.
    (Beam ?beam) ;; Static - List of all Beam ID
    (BeamAtStorage ?beam)
    (BeamAtRobot ?beam)
    (BeamAtAssembled ?beam)

    ; (Gripper ?gripper) ;; Static - List of all Gripper ID
    ; There are two gripper position states: AtStorage, AtRobot. One and Only One is True at the same time.
    (GripperAtRobot ?gripper)
    (GripperAtStorage ?gripper)

    ; Conditions where a beam requires a certain gripper type
    (BeamNeedsGripperType ?beam ?grippertype) ;; Static - List of all gripper types (can be multiple) required for each beam
    (GripperOfType ?gripper ?grippertype) ;; Static - Statement describing the type of a gripper

    ;; Joints are defined as the intersection of two beams
    ;; Joints are implied to have order, so (Joint ?beam1 ?beam2) is not the same as (Joint ?beam2 ?beam1)
    ;; ?beam1 have to be assembled before ?beam2 if (Joint ?beam1 ?beam2) is declared
    (Joint ?beam1 ?beam2) ;; Static - List of all joints (beam_id, beam_id)

    ;; Predicates certified by the streams
    (AssembleBeamTraj ?beam ?traj)
    (AssembleBeamNotInCollision ?traj ?heldbeam ?otherbeam)
  )
  

    (:action assemble_beam_with_gripper
        :parameters (?beam ?gripper ?grippertype ?traj)
        :precondition (and
            ;; Beam is at storage
            (BeamAtStorage ?beam)

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
                        ;; and the eariler beam is not yet assembled
                        (not (BeamAtAssembled ?earlierbeam))
                    ))
            )

            (AssembleBeamTraj ?beam ?traj)
            (not
              (exists (?otherbeam) (and 
                  (BeamAtAssembled ?otherbeam)
                  (not (AssembleBeamNotInCollision ?traj ?beam ?otherbeam))
                                     ))
            )
            ; (forall (?otherbeam) (imply 
            ;     (BeamAtAssembled ?otherbeam)
            ;     (AssembleBeamNotInCollision ?traj ?beam ?otherbeam)
            ;                        ))

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
        ;; Robot is not currently holding a gripper
        (not
            (exists
                (?anygripper)
                (GripperAtRobot ?anygripper))) 
    )
    :effect (and
      (not (GripperAtStorage ?gripper)) ;; Gripper no longer at storage
      (GripperAtRobot ?gripper) ;; Gripper now at robot
    )
  )

  (:action place_gripper_to_storage
    :parameters (?gripper)
    :precondition (and
        ;; gripper is a valid Gripper
        ; (Gripper ?gripper) 
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

)