(define (problem simple_beam_assembly)
    (:domain itj_clamp_only)
    (:objects 
        beam1
        beam2
        beam3
        gripper1 
        gripper2
        grippertypeA
        grippertypeB
        clamp1
        clamp2
        clamp3
        clamp4
        clamptypeA
        clamptypeB
    )

    (:init 
        ; Initial state for beams
        (Beam beam1)
        (Beam beam2)
        (Beam beam3)

        (BeamAtStorage beam1)
        (BeamAtStorage beam2)
        (BeamAtStorage beam3)

        ; Initial state for grippers
        (Gripper gripper1)
        (Gripper gripper2)
        
        (GripperAtStorage gripper1)
        (GripperAtStorage gripper2)

        ; Declaring gripper types
        (GripperOfType gripper1 grippertypeA)
        (GripperOfType gripper2 grippertypeB)

        ; Beam gripper requirements
        (BeamNeedsGripperType beam1 grippertypeA)
        (BeamNeedsGripperType beam2 grippertypeB)
        (BeamNeedsGripperType beam3 grippertypeB)
        
        ; Initial state for clamps
        (ClampAtStorage clamp1)
        (ClampAtStorage clamp2)
        (ClampAtStorage clamp3)
        (ClampAtStorage clamp4)
        
        ; Declaring clamps
        (Clamp clamp1)
        (Clamp clamp2)
        (Clamp clamp3)
        (Clamp clamp4)
        
        ; Declaring clamp types
        (ClampOfType clamp1 clamptypeA)
        (ClampOfType clamp2 clamptypeA)
        (ClampOfType clamp3 clamptypeB)
        (ClampOfType clamp4 clamptypeB)
        
        ; Declaring joints
        (Joint beam1 beam2)
        (Joint beam1 beam3)
        (Joint beam2 beam3)
        
        ; Beam clamp requirements
        (JointNeedsClamp beam1 beam2 clamptypeA)
        (JointNeedsClamp beam1 beam3 clamptypeA)
        (JointNeedsClamp beam2 beam3 clamptypeB)

    )

    (:goal 
        (and
            ; Set the goal to have the beam assembled
            (BeamAtAssembled beam1)
            (BeamAtAssembled beam2)
            (BeamAtAssembled beam3)

            ; ... and both grippers stored
            (GripperAtStorage gripper1)
            (GripperAtStorage gripper2)
            
            (ClampAtStorage clamp1)
            (ClampAtStorage clamp2)
            (ClampAtStorage clamp3)
            (ClampAtStorage clamp4)
        )
    )
)
