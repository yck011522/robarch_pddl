(define (stream itj_gripper_ik)
    (:stream assemble_beam_ik_check
        :inputs (?beam ?gripper)
        :domain (and
            (Beam ?beam)
            (Gripper ?gripper)
            (BeamGripperTypeMatched ?beam ?gripper)
        )
        :fluents (BeamAtStorage BeamAtRobot BeamAtAssembled GripperAtRobot GripperAtStorage)
        :outputs (?ikresult)
        :certified (AssembleBeamIkReachable ?beam ?gripper ?ikresult)
    )
)