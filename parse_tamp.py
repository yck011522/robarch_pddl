import os
from collections import defaultdict

import load_pddlstream
from pddlstream.utils import read, write
from pddlstream.language.stream import DEBUG

from pddlstream.language.constants import PDDLProblem
from pddlstream.language.constants import And, Equal, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test

from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.planning.robot_setup import load_RFL_world, get_tolerances
from integral_timber_joints.planning.state import set_state, set_initial_state
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.process.action import BeamPlacementWithClampsAction, BeamPlacementWithoutClampsAction, AssembleBeamWithScrewdriversAction, RoboticMovement

from load_pddlstream import HERE
from parse_symbolic import process_to_init_goal_by_case
from stream_samplers import sample_ik_for_target_frames
from utils import LOGGER

def get_pddlstream_problem(
        process: RobotClampAssemblyProcess,
        case_number: int,
        num_elements_to_export: int,
        pddl_folder: str,
        enable_stream=True):
    """Convert a Process instance into a PDDLStream formulation
    """
    options = options or {}
    debug = options.get('debug', False)
    diagnosis = options.get('diagnosis', False)

    domain_pddl = read(os.path.join(pddl_folder, 'domain.pddl'))
    stream_pddl = read(os.path.join(HERE, pddl_folder, 'stream.pddl'))

    init, goal = process_to_init_goal_by_case(
        process, case_number, [], [], num_elements_to_export=num_elements_to_export)

    if enable_stream:
        # * Connect to path planning backend and initialize robot parameters
        client, robot, _ = load_RFL_world(viewer=diagnosis)
        # * initialize collision objects and tools in the scene
        assert set_initial_state(client, robot, process, initialize=True, options=options), 'Setting initial state failed.'

        stream_map = {
            'plan_motion_for_beam_assembly':  from_fn(get_sample_fn_plan_motion_for_beam_assembly(client, robot, process, options=options)),
            'beam_assembly_collision_check': from_fn(get_test_fn_beam_assembly_collision_check(client, robot, process, options=options)),
        }
    else:
        stream_map = DEBUG

    constant_map = {}
    pddlstream_problem = PDDLProblem(
        domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return pddlstream_problem

def get_sample_fn_plan_motion_for_beam_assembly(client, robot, process, options=None):
    options = options or {}

    # Precompute and cache all target frames for each beam's assembly action
    gripper_names_from_type = defaultdict(list)
    for gripper in process.grippers:
        gripper_names_from_type[gripper.type_name].append(gripper.name)

    target_frames_from_beam_id = defaultdict(list)
    for i, beam_id in enumerate(process.assembly.sequence):
        gripper_type = process.assembly.get_beam_attribute(
            beam_id, "gripper_type")
        # randomly assign one suitable gripper to each beam just for sampling purposes
        gripper_id = gripper_names_from_type[gripper_type][0]

        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.GROUND_CONTACT:
            action = BeamPlacementWithoutClampsAction(beam_id=beam_id, gripper_id=gripper_id)

        elif process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.CLAMPED:
            # we can leave joint_ids and clamp_ids empty beacuse we don't need ACM here
            action = BeamPlacementWithClampsAction(beam_id=beam_id, joint_ids=[], gripper_id=gripper_id, clamp_ids=[])

        elif (process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITH_GRIPPER or \
              process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER):
            # we can leave joint_ids and clamp_ids empty beacuse we don't need ACM here
            action = AssembleBeamWithScrewdriversAction(beam_id=beam_id, joint_ids=[], gripper_id=gripper_id, clamp_ids=[])

        else:
            continue

        # * compute movements from action
        action.create_movements(process)
        for movement in action.movements:
            # * trigger state diff computation
            movement.create_state_diff(process)

            # * skip non-robotic movements for IK computation
            if not isinstance(movement, RoboticMovement):
               continue

            # * skip if there exists a taught conf
            if movement.target_configuration is not None:
                continue

            # * compute IK
            if movement.target_frame is None:
                LOGGER.error(f'Target frame is None in {movement.short_summary}')

            end_t0cf_frame = movement.target_frame.copy()
            end_t0cf_frame.point *= 1e-3
            target_frames_from_beam_id[beam_id].append(end_t0cf_frame)
 
    def sample_fn(beam_id: str, gripper_type: str):
        return sample_ik_for_target_frames(client, robot, target_frames_from_beam_id[beam_id], options=options)

    return sample_fn

def get_test_fn_beam_assembly_collision_check(client, robot, process, options=None):
    options = options or {}
    # cache all beam's frame at asssembled position

    def test_fn(traj, heldbeam: str, otherbeam: str):
        # set the otherbeam to the assembled position
        client.set_object_frame('^{}$'.format(otherbeam), heldbeam_frame_atassembled)

        for conf in traj.points:
            # TODO: could shuffle this
            # check robot body collision with the otherbeam
            if pairwise_collision(mov, body):
                return True

            # check heldbeam collision with the otherbeam using FK

            # TODO check robot-attached object with the otherbeam

        return False

    return test_fn