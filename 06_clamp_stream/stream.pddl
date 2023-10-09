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
  
;   (:stream plan_motion_for_attach_clamp
;     :inputs (?heldclamp ?clamptype ?beam1 ?beam2)
;     :domain (and 
;         ;; Gripper type matching:
;         ; (BeamNeedsGripperType ?beam ?grippertype)
;         )
;     :outputs (?traj)
;     :certified (AssembleBeamTraj ?beam ?traj)
;   )
  
;   (:stream plan_motion_for_detach_clamp
;     :inputs (?heldclamp ?clamptype ?beam1 ?beam2)
;     :domain (and 
;         ;; Gripper type matching:
;         ; (BeamNeedsGripperType ?beam ?grippertype)
;         )
;     :outputs (?traj)
;     :certified (AssembleBeamTraj ?beam ?traj)
;   )

  (:stream beam_assembly_collision_check
    :inputs (?traj ?heldbeam ?otherbeam)
    :domain (and 
        (AssembleBeamTraj ?heldbeam ?traj) 
        (Beam ?otherbeam)
        )
    :certified (AssembleBeamNotInCollision ?traj ?heldbeam ?otherbeam)
  )
  
;   (:stream clamp_clamp_collision_check
;     :inputs (?traj ?heldclamp ?otherclamp)
;     :domain (and 
;         ; (AssembleBeamTraj ?heldbeam ?traj) 
;         ; (Beam ?otherbeam)
;         )
;     :certified (AssembleBeamNotInCollision ?traj ?heldbeam ?otherbeam)
;   )

;   (:stream clamp_beam_collision_check
;     :inputs (?traj ?heldclamp ?otherbeam)
;     :domain (and 
;         ; (AssembleBeamTraj ?heldbeam ?traj) 
;         ; (Beam ?otherbeam)
;         )
;     :certified (AssembleBeamNotInCollision ?traj ?heldbeam ?otherbeam)
;   )
)