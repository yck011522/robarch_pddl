(define (stream clamp_transfer)
  (:stream plan_motion_for_beam_assembly
    :inputs (?beam ?grippertype)
    :domain (and 
        ;; Gripper type matching:
        (BeamNeedsGripperType ?beam ?grippertype)
        )
    :outputs (?traj)
    :certified (AssembleBeamTraj ?beam ?traj)
  )
  
  (:stream plan_motion_for_attach_clamp
    :inputs (?heldclamp ?clamptype ?beam1 ?beam2)
    :domain (and 
        (ClampOfType ?heldclamp ?clamptype)
        (JointNeedsClampType ?beam1 ?beam2 ?clamptype)
        )
    :outputs (?traj)
    :certified (and 
        (AttachClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
        (ClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
    )
  )
  
  (:stream plan_motion_for_detach_clamp
    :inputs (?heldclamp ?clamptype ?beam1 ?beam2)
    :domain (and 
        (ClampOfType ?heldclamp ?clamptype)
        (JointNeedsClampType ?beam1 ?beam2 ?clamptype)
        )
    :outputs (?traj)
    :certified (and 
        (DetachClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
        (ClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
    )
  )

  (:stream beam_assembly_collision_check
    :inputs (?traj ?heldbeam ?otherbeam)
    :domain (and 
        (AssembleBeamTraj ?heldbeam ?traj) 
        (Beam ?otherbeam)
        )
    :certified (AssembleBeamNotInCollision ?traj ?heldbeam ?otherbeam)
  )
  
  (:stream clamp_clamp_collision_check
    :inputs (?traj ?heldclamp ?beam1 ?beam2 ?otherclamp)
    :domain (and 
        (ClampTraj ?heldclamp ?beam1 ?beam2 ?traj) 
        (Clamp ?otherclamp)
        )
    :certified (ClampTrajNotInCollisionWithClamp ?traj ?heldclamp ?beam1 ?beam2 ?otherclamp)
  )

  (:stream clamp_beam_collision_check
    :inputs (?traj ?heldclamp ?beam1 ?beam2 ?otherbeam)
    :domain (and 
        (ClampTraj ?heldclamp ?beam1 ?beam2 ?traj) 
        (Beam ?otherbeam)
        )
    :certified (ClampTrajNotInCollisionWithBeam ?traj ?heldclamp ?beam1 ?beam2 ?otherbeam)
  )
)