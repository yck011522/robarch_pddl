import os
from collections import defaultdict

import pybullet_planning as pp
from compas_fab.robots import Robot, AttachedCollisionMesh, CollisionMesh
from compas_fab_pychoreo.client import PyChoreoClient

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
        enable_stream=True,
        options=None):
    """Convert a Process instance into a PDDLStream formulation
    """
    options = options or {}
    debug = options.get('debug', True)
    viewer = options.get('viewer', False)
    diagnosis = options.get('diagnosis', False)

    domain_pddl = read(os.path.join(pddl_folder, 'domain.pddl'))
    stream_pddl = read(os.path.join(HERE, pddl_folder, 'stream.pddl'))

    init, goal = process_to_init_goal_by_case(
        process, case_number, [], [], num_elements_to_export=num_elements_to_export)

    if enable_stream:
        # * Connect to path planning backend and initialize robot parameters
        client, robot, _ = load_RFL_world(viewer=viewer or diagnosis, verbose=True)

        # frame, conf compare, joint flip and allowable collision tolerances are set here
        options.update(get_tolerances(robot))

        # * initialize collision objects and tools in the scene
        assert set_initial_state(client, robot, process, initialize=True, options=options), 'Setting initial state failed.'

        stream_map = {
            'plan_motion_for_beam_assembly':  from_fn(get_sample_fn_plan_motion_for_beam_assembly(client, robot, process, options=options)),
            'beam_assembly_collision_check': from_test(get_test_fn_beam_assembly_collision_check(client, robot, process, options=options)),
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
        if gripper.type_name:
            gripper_names_from_type[gripper.type_name].append(gripper.name)

    target_frames_from_beam_id = defaultdict(list)
    for i, beam_id in enumerate(process.assembly.sequence):
        #  Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue

        gripper_type = process.assembly.get_beam_attribute(
            beam_id, "gripper_type")
        assert gripper_type in gripper_names_from_type
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
            action = AssembleBeamWithScrewdriversAction(beam_id=beam_id, joint_ids=[], gripper_id=gripper_id, screwdriver_ids=[])

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

def get_test_fn_beam_assembly_collision_check(
        client: PyChoreoClient, 
        robot: Robot, 
        process: RobotClampAssemblyProcess,
        options=None):
    options = options or {}

    toolchanger = process.robot_toolchanger
    flange_from_toolchanger_base = toolchanger.t_t0cf_from_tcf
    beam_assembled_frames = {}
    beam_grasps = {}
    for beam_id in process.assembly.sequence:
        # Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue

        # Cache all beam's frame at asssembled position
        assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_final')
        beam_assembled_frames[beam_id] = assembly_wcf_final

        # Cache all grasp transformations
        # ? different gripper might have different grasp for a beam?
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(beam_id)
        beam_gripper_id = process.assembly.get_beam_attribute(beam_id, "gripper_id")
        beam_gripper = process.tool(beam_gripper_id)
        flange_from_beam = flange_from_toolchanger_base * beam_gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam
        beam_grasps[beam_id] = flange_from_beam

    flange_link_name = process.ROBOT_END_LINK
    touched_robot_links = []
    attached_object_base_link_name = None

    def test_fn(traj, heldbeam: str, otherbeam: str):
        # Returns: AssembleBeamNotInCollision
        # set the otherbeam to the assembled position
        assemble_beam_not_in_collision = False
        with pp.WorldSaver():
            client.set_object_frame('^{}$'.format(otherbeam), beam_assembled_frames[otherbeam])

            # Create attachments for the heldbeam
            client.add_attached_collision_mesh(
                AttachedCollisionMesh(CollisionMesh(None, heldbeam),
                                      flange_link_name, touch_links=touched_robot_links),
                options={'robot': robot,
                         'attached_child_link_name': attached_object_base_link_name,
                         'parent_link_from_child_link_transformation' : beam_grasps[heldbeam],
                         })

            for conf in traj.points:
                # TODO: could shuffle this

                # check robot and heldbeam collision with the otherbeam using FK
                # ! this checks a lot more than what we need here
                # so there are rooms for acceleration, but the code will get more complicated
                assemble_beam_not_in_collision = not client.check_collisions(robot, conf, options=options)

                if not assemble_beam_not_in_collision:
                    # early return if collision found
                    break

        # clean up the attached object
        client.detach_attached_collision_mesh(heldbeam, options={})
        return assemble_beam_not_in_collision

    return test_fn