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
    (GripperOfType ?gripper ?grippertype) ;; Static - Statement describing the type of a gripper
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
      (GripperAtRobot ?gripper) ;; Gripper is already on robot
      (BeamAtRobot ?beam) ;; Beam is already on robot
    )
    :effect (and
      (not (BeamAtRobot ?beam)) ;; Beam no longer at storage
      (BeamAtAssembled ?beam) ;; Beam now at robot
    )
  )

  (:action pick_gripper_from_storage
    :parameters (?gripper)
    :precondition (and
        ;; Robot is not currently holding a gripper
        (not
            (exists
                (?anygripper)
                (GripperAtRobot ?anygripper))) 
        ;; Robot is not currently holding a beam
        (not
            (exists
                (?beam)
                (BeamAtRobot ?beam))) 
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

)