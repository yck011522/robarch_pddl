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

  (:stream beam_assembly_collision_check
    :inputs (?traj ?heldbeam ?otherbeam)
    :domain (and 
        (AssembleBeamTraj ?heldbeam ?traj) 
        (Beam ?otherbeam)
        )
    :certified (AssembleBeamNotInCollision ?traj ?heldbeam ?otherbeam)
  )
    
  (:stream plan_motion_for_attach_clamp
    :inputs (?heldclamp ?clamptype ?beam1 ?beam2)
    :domain (and 
        (ClampOfType ?heldclamp ?clamptype)
        (JointNeedsClampType ?beam1 ?beam2 ?clamptype)
        )
    :outputs (?traj)
    :certified (AttachClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
        ; (ClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
  )
  
  (:stream plan_motion_for_detach_clamp
    :inputs (?heldclamp ?clamptype ?beam1 ?beam2)
    :domain (and 
        (ClampOfType ?heldclamp ?clamptype)
        (JointNeedsClampType ?beam1 ?beam2 ?clamptype)
        )
    :outputs (?traj)
    :certified (DetachClampTraj ?heldclamp ?beam1 ?beam2 ?traj)
  )

;   (:stream attach_clamp_clamp_collision_check
;     :inputs (?heldclamp ?beam1 ?beam2 ?traj ?otherclamp ?otherbeam1 ?otherbeam2 ?otherclamptype)
;     :domain (and 
;         (AttachClampTraj ?heldclamp ?beam1 ?beam2 ?traj) 
;         (Clamp ?otherclamp)
;         (Joint ?otherbeam1 ?otherbeam2)
;         (ClampOfType ?otherclamp ?otherclamptype)
;         (JointNeedsClampType ?otherbeam1 ?otherbeam2 ?otherclamptype)
;         )
;     :certified (AttachClampTrajNotInCollisionWithClamp ?heldclamp ?beam1 ?beam2 ?traj ?otherclamp ?otherbeam1 ?otherbeam2)
;   )

;   (:stream detach_clamp_clamp_collision_check
;     :inputs (?heldclamp ?beam1 ?beam2 ?traj ?otherclamp ?otherbeam1 ?otherbeam2 ?otherclamptype)
;     :domain (and 
;         (DetachClampTraj ?heldclamp ?beam1 ?beam2 ?traj) 
;         (Clamp ?otherclamp)
;         (Joint ?otherbeam1 ?otherbeam2)
;         (ClampOfType ?otherclamp ?otherclamptype)
;         (JointNeedsClampType ?otherbeam1 ?otherbeam2 ?otherclamptype)
;         )
;     :certified (DetachClampTrajNotInCollisionWithClamp ?heldclamp ?beam1 ?beam2 ?traj ?otherclamp ?otherbeam1 ?otherbeam2)
;   )

  (:stream attach_clamp_beam_collision_check
    :inputs (?heldclamp ?beam1 ?beam2 ?traj ?otherbeam)
    :domain (and 
        (AttachClampTraj ?heldclamp ?beam1 ?beam2 ?traj) 
        (Beam ?otherbeam)
        )
    :certified (AttachClampTrajNotInCollisionWithBeam ?heldclamp ?beam1 ?beam2 ?traj ?otherbeam)
  )

  (:stream detach_clamp_beam_collision_check
    :inputs (?heldclamp ?beam1 ?beam2 ?traj ?otherbeam)
    :domain (and 
        (DetachClampTraj ?heldclamp ?beam1 ?beam2 ?traj) 
        (Beam ?otherbeam)
        )
    :certified (DetachClampTrajNotInCollisionWithBeam ?heldclamp ?beam1 ?beam2 ?traj ?otherbeam)
  )
)