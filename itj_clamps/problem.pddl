(define (problem simple_beam_assembly)
    (:domain itj_clamp_only)
    (:objects 
        beam1
        beam2
        beam3
        beam4
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
        ;; Static List of each Beam
        (Beam beam1)
        (Beam beam2)
        (Beam beam3)
        (Beam beam4)

        ;; Initial position of each beam, by deafult it is AtStorage
        (BeamAtStorage beam1)
        (BeamAtStorage beam2)
        (BeamAtStorage beam3)
        (BeamAtStorage beam4)

        ;; Static List of each Gripper
        (Gripper gripper1)
        (Gripper gripper2)
        
        ;; Initial position of each gripper, by deafult it is AtStorage
        (GripperAtStorage gripper1)
        (GripperAtStorage gripper2)

        ;; Declaring gripper types, one for each gripper
        (GripperOfType gripper1 grippertypeA)
        (GripperOfType gripper2 grippertypeB)

        ;; Gripper requirements for picking up beam, at least one gripper type for each beam
        ;; If more than one BeamNeedsGripperType is declared for a beam, it means that either gripper type can be used
        (BeamNeedsGripperType beam1 grippertypeA)
        (BeamNeedsGripperType beam2 grippertypeB)
        (BeamNeedsGripperType beam3 grippertypeB)
        
        (BeamNeedsGripperType beam4 grippertypeA) ;; Either Type A or Type B can be used
        (BeamNeedsGripperType beam4 grippertypeB) ;; Either Type A or Type B can be used
        
        ;; Initial position of each clamp, by deafult it is AtStorage
        (ClampAtStorage clamp1)
        (ClampAtStorage clamp2)
        (ClampAtStorage clamp3)
        (ClampAtStorage clamp4)
        
        ;; Static List of each Clamp
        (Clamp clamp1)
        (Clamp clamp2)
        (Clamp clamp3)
        (Clamp clamp4)
        
        ;; Declaring clamp types, one for each clamp
        (ClampOfType clamp1 clamptypeA)
        (ClampOfType clamp2 clamptypeA)
        (ClampOfType clamp3 clamptypeB)
        (ClampOfType clamp4 clamptypeB)
        
        ;; Declaring joints that needs to be assembled (be aware of the order of the beams)
        (Joint beam1 beam2)
        (Joint beam1 beam3)
        (Joint beam2 beam3)
        (Joint beam2 beam4)
        (Joint beam3 beam4)
        
        ;; Clamp type requirements for specific joints
        ;; If no JointNeedsClamp is declared for a joint, it is assumed that no clamp is needed
        ;; If more than one JointNeedsClamp is declared for a joint, it is assumed that either clamp type can be used
        (JointNeedsClamp beam1 beam2 clamptypeA)
        (JointNeedsClamp beam1 beam3 clamptypeA)

        (JointNeedsClamp beam2 beam3 clamptypeA) ;; Either Type A or Type B can be used
        (JointNeedsClamp beam2 beam3 clamptypeB) ;; Either Type A or Type B can be used

        (JointNeedsClamp beam2 beam4 clamptypeA)
        (JointNeedsClamp beam3 beam4 clamptypeB)

    )

    (:goal 
        (and
            ; Main goal is to have all beams assembled
            (BeamAtAssembled beam1)
            (BeamAtAssembled beam2)
            (BeamAtAssembled beam3)
            (BeamAtAssembled beam4)

            ; Side goal is to have all grippers and clamps moved back to storage
            (GripperAtStorage gripper1)
            (GripperAtStorage gripper2)
            
            (ClampAtStorage clamp1)
            (ClampAtStorage clamp2)
            (ClampAtStorage clamp3)
            (ClampAtStorage clamp4)
        )
    )
)
