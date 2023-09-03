(define (domain itj_gripper_only)
  (:requirements :negative-preconditions :derived-predicates :strips :equality)
  (:predicates
    ; There are thee beam position states: AtStorage, AtRobot, and AtAssembled. One and Only One is True at the same time.
    (Beam ?beam) ;; Static - List of all Beam ID
    (BeamAtStorage ?beam)
    (BeamAtRobot ?beam)
    (BeamAtAssembled ?beam)
    (RobotIsHoldingBeam) ;; Derived - True if robot is holding a beam

    (Gripper ?gripper) ;; Static - List of all Gripper ID
    ; There are two gripper position states: AtStorage, AtRobot. One and Only One is True at the same time.
    (GripperAtRobot ?gripper)
    (GripperAtStorage ?gripper)
    (RobotIsHoldingGripper ?gripper) ;; Derived - True if robot is holding a gripper

    ; Conditions where a beam requires a certain gripper type
    (BeamNeedsGripperType ?beam ?grippertype) ;; Static - List of all gripper types (can be multiple) required for each beam
    (GripperType ?grippertype) ;; Static - List of all gripper types
    (GripperOfType ?gripper ?grippertype) ;; Static - Statement describing the type of a gripper
    (BeamGripperTypeMatched ?beam ?gripper) ;; Derived - True if beam and gripper are matched
  )

  (:action pick_beam
    :parameters (?beam ?gripper)
    :precondition (and
      (Beam ?beam) ;; ?beam is a valid Beam
      (BeamAtStorage ?beam) ;; Beam is at storage
      (GripperAtRobot ?gripper) ;; Gripper is already on robot
      (BeamGripperTypeMatched ?beam ?gripper) ;; Beam requires a certain gripper type
    )
    :effect (and
      (not (BeamAtStorage ?beam)) ;; Beam no longer at storage
      (BeamAtRobot ?beam) ;; Beam at robot
      ; (increase (total-cost) 1)
    )
  )

  (:action assemble_beam
    :parameters (?beam ?gripper)
    :precondition (and
      (GripperAtRobot ?gripper) ;; Gripper is already on robot
      (BeamAtRobot ?beam) ;; Beam is already on robot
    )
    :effect (and
      (not (BeamAtRobot ?beam)) ;; Beam no longer at robot
      (BeamAtAssembled ?beam) ;; Beam now at assembled
      ; (increase (total-cost) 1)
    )
  )

  (:action pick_gripper_from_storage
    :parameters (?gripper)
    :precondition (and
      (not (RobotIsHoldingBeam)) ;; Robot is not currently holding a beam
      (not (RobotIsHoldingGripper)) ;; Robot is not currently holding a gripper
      (Gripper ?gripper) ;; ?gripper is a valid Gripper
    )
    :effect (and
      (not (GripperAtStorage ?gripper)) ;; Gripper no longer at storage
      (GripperAtRobot ?gripper) ;; Gripper now at robot
      ; (increase (total-cost) 1)
    )
  )

  (:action place_gripper_to_storage
    :parameters (?gripper)
    :precondition (and
      (Gripper ?gripper) ;; ?gripper is a valid Gripper
      (GripperAtRobot ?gripper) ;; Robot is currently holding ?gripper
      (not (RobotIsHoldingBeam)) ;; Robot is not currently holding a beam
    )
    :effect (and
      (not (GripperAtRobot ?gripper)) ;; Gripper no longer at robot
      (GripperAtStorage ?gripper) ;; Gripper now at storage
      ; (increase (total-cost) 1)
    )
  )

  ; Derived predicates

  (:derived
    (RobotIsHoldingBeam) ;; True if robot is holding a gripper
    (exists
      (?beam)
      (BeamAtRobot ?beam))
  )

  (:derived
    (RobotIsHoldingGripper) ;; True if robot is holding a gripper
    (exists
      (?gripper)
      (GripperAtRobot ?gripper))
  )

  (:derived
    (BeamGripperTypeMatched ?beam ?gripper) ;; True if robot is holding a gripper
    (exists
      (?grippertype)
      (and
        (Beam ?beam) ;; (type check)
        (Gripper ?gripper) ;; (type check)
        (GripperType ?grippertype) ;; (type check)
        (GripperOfType ?gripper ?grippertype)
        (BeamNeedsGripperType ?beam ?grippertype))
    )
  )

)