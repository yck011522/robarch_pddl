(define (problem simple_beam_assembly)
    (:domain itj_gripper_only)
    (:objects 
        beam1
        beam2
        gripper1 
        gripper2
        grippertypeA
        grippertypeB
    )

    (:init 
        ; Initial state for beams
        ; (Beam beam1)
        ; (Beam beam2)
        (BeamAtStorage beam1)
        (BeamAtStorage beam2)
        ; Initial state for grippers
        ; (Gripper gripper1)
        ; (Gripper gripper2)
        (GripperAtStorage gripper1)
        (GripperAtStorage gripper2)

        ; Initial state for gripper types
        (GripperOfType gripper1 grippertypeA)
        (GripperOfType gripper2 grippertypeB)

        ; Initial state for beam requirements
        (BeamNeedsGripperType beam1 grippertypeA)
        (BeamNeedsGripperType beam2 grippertypeB)
    )

    (:goal 
        (and
            ; Set the goal to have the beam assembled
            (BeamAtAssembled beam1)
            (BeamAtAssembled beam2)
            ; ... and both grippers stored
            (GripperAtStorage gripper1)
            (GripperAtStorage gripper2)
        )
    )
)
