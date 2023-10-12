PDDL_ACTIONS_ITJ_ACTIONS = {
    'assemble_beam_by_clamping_method': [
        'LoadBeamAction',
        'PickBeamWithGripperAction',
        'BeamPlacementWithClampsAction',
        'RetractGripperFromBeamAction',
    ],
    'assemble_beam_by_screwing_method': [
        'LoadBeamAction',
        'PickScrewdriverFromStorageAction',
        'CloseGripperOnBeamAction',
        'GenericGripperApproachBeamPickupAction',
        'PickAndRotateBeamForAttachingScrewdriverAction',
        'OperatorAttachScrewdriverAction',
        'AssembleBeamWithScrewdriversAction',
        'RetractScrewdriverFromBeamAction',
        'PlaceScrewdriverToStorageAction'
    ],
    'assemble_beam_by_ground_connection': [
        'LoadBeamAction',
        'BeamPlacementWithoutClampsAction',
    ],
    'pick_gripper_from_storage': ['PickGripperFromStorageAction'],
    'place_gripper_to_storage': ['PlaceGripperToStorageAction'],
    'pick_clamp_from_storage': ['PickClampFromStorageAction'],
    'place_clamp_to_storage': ['PlaceClampToStorageAction'],
    'attach_clamp_to_joint': ['PlaceClampToStructureAction'],
    'detach_clamp_from_joint': ['PickClampFromStructureAction'],
}

PDDL_ACTIONS_TYPE_GROUPS = {
    'assemble_action': ['assemble_beam_by_ground_connection', 'assemble_beam_by_clamping_method', 'assemble_beam_by_screwing_method'],
    'gripper_action': ['pick_gripper_from_storage', 'place_gripper_to_storage'],
    'clamp_action': ['pick_clamp_from_storage', 'place_clamp_to_storage', 'attach_clamp_to_joint', 'detach_clamp_from_joint'],
}
