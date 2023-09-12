(define (stream itj_gripper_collision)

  (:stream plan_motion_for_beam_assembly
    :inputs (?beam ?grippertype)
    :domain (and 
        ;; Gripper type matching:
        (BeamNeedsGripperType ?beam ?grippertype)
        )
    :outputs (?traj)
    :certified (AssembleBeamTraj ?beam ?traj)
  )

  (:stream beam_assembly_collision_check
    :inputs (?traj ?heldbeam ?otherbeam)
    :domain (and 
        (AssembleBeamTraj ?heldbeam ?traj) 
        (Beam ?otherbeam)
        )
    :certified (AssembleBeamNotInCollision ?traj ?heldbeam ?otherbeam)
  )

  ;To Attach clamp to a joint (Test Collision: robot empty hand - other beams)
  (:stream attach_clamp_collision_check
    :inputs (?traj ?clamp ?beam1 ?beam2 ?otherbeam)
    :domain (and 
        (AttachClampTraj ?clamp ?traj) 
        (Beam ?otherbeam)
        (Clamp ?targetclamp)
        (Joint ?beam1 ?beam2)
        )
    :certified (AttachClampNotInCollision ?traj ?heldbeam ?otherbeam)
  )

  ;To Detach clamp to a joint (robot with clamp in hand - beams / other clamps)
  (:stream detach_clamp_collision_check_attach
    :inputs (?traj ?clamp ?beam1 ?beam2 ?otherbeam)
    :domain (and 
        (DetachClampTraj ?clamp ?traj) 
        (Beam ?otherbeam)
        (Clamp ?clamp)
        (Joint ?beam1 ?beam2)
        )
    :certified (DetachClampNotInCollision ?traj ?heldbeam ?otherbeam)
  )
)