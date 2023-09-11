(define (stream itj_gripper_collision)

  (:stream plan_motion_for_beam_assembly
    :inputs (?beam ?grippertype)
    :domain (and 
        ;; Gripper type matching:
        (BeamNeedsGripperType ?beam ?grippertype)
        )
    :outputs (?traj)
    :certified (BeamAssemblyTraj ?beam ?traj)
  )

  (:stream beam_assembly_collision_check
    :inputs (?traj ?heldbeam ?otherbeam)
    :domain (and 
        (BeamAssemblyTraj ?heldbeam ?traj) 
        (Beam ?otherbeam)
        )
    :certified (BeamAssemblyNotInCollision ?traj ?heldbeam ?otherbeam)
  )

)