from gc import enable
import os
from unittest import case
from ext.pddlstream.pddlstream.language.generator import from_gen

import load_pddlstream
from pddlstream.utils import read, write
from pddlstream.language.stream import DEBUG

from pddlstream.language.constants import PDDLProblem
from pddlstream.language.constants import And, Equal, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test, from_sampler

from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.planning.robot_setup import load_RFL_world, get_tolerances
from integral_timber_joints.planning.state import set_state, set_initial_state
from load_pddlstream import HERE
from parse_symbolic import process_to_init_goal_by_case, export_pddl
from stream_samplers import get_sample_fn_plan_motion_for_beam_assembly, get_test_fn_beam_assembly_collision_check, \
    get_sample_fn_plan_motion_for_clamp, get_test_fn_clamp_clamp_collision_check, get_test_fn_clamp_beam_collision_check
from utils import LOGGER, print_pddl_task_object_names

def get_pddlstream_problem(
        process: RobotClampAssemblyProcess,
        case_number: int,
        num_elements_to_export: int,
        pddl_folder: str,
        enable_stream=True,
        options=None,
        problem_name = 'problem_name'):
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
    
    # Export PDDL domain file
    export_pddl('domain_name', init,
        goal, pddl_folder, problem_name)
    
    if case_number in [1,2,3,5] and enable_stream:
        enable_stream = False
        LOGGER.warning('Case {} is a symbolic-only domain, stream is disabled.'.format(case_number))

    if enable_stream:
        # * Connect to path planning backend and initialize robot parameters
        client, robot, _ = load_RFL_world(viewer=viewer or diagnosis, verbose=True)

        # frame, conf compare, joint flip and allowable collision tolerances are set here
        options.update(get_tolerances(robot))

        # * initialize collision objects and tools in the scene
        assert set_initial_state(client, robot, process, initialize=True, options=options), 'Setting initial state failed.'

        stream_map = {}
        if case_number == 4:
            stream_map.update(get_beam_assembly_streams(client, robot, process, options))
        elif case_number == 6 or case_number == 7:
            stream_map.update(get_beam_assembly_streams(client, robot, process, options))
            stream_map.update(get_clamp_transfer_streams(client, robot, process, options))
    else:
        stream_map = DEBUG

    constant_map = {}
    pddlstream_problem = PDDLProblem(
        domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    print_pddl_task_object_names(pddlstream_problem)

    return pddlstream_problem

from stream_samplers_stateless import get_sample_fn_plan_motion_for_beam_assembly_stateless, get_test_fn_beam_assembly_collision_check_stateless

def get_beam_assembly_streams(client, robot, process, options):
    return {
            # 'plan_motion_for_beam_assembly':  from_gen_fn(get_sample_fn_plan_motion_for_beam_assembly_stateless(client, robot, process, options=options)),
            # 'beam_assembly_collision_check': from_test(get_test_fn_beam_assembly_collision_check_stateless(client, robot, process, options=options)),

            'plan_motion_for_beam_assembly':  from_sampler(get_sample_fn_plan_motion_for_beam_assembly(client, robot, process, options=options)),
            'beam_assembly_collision_check': from_test(get_test_fn_beam_assembly_collision_check(client, robot, process, options=options)),
        }

def get_clamp_transfer_streams(client, robot, process, options):
    return {
            'plan_motion_for_attach_clamp':  from_sampler(get_sample_fn_plan_motion_for_clamp(client, robot, process, operation='attach', options=options)),
            'plan_motion_for_detach_clamp':  from_sampler(get_sample_fn_plan_motion_for_clamp(client, robot, process, operation='detach', options=options)),

            # 'attach_clamp_clamp_collision_check': from_test(get_test_fn_clamp_clamp_collision_check(client, robot, process, options=options)),
            # 'detach_clamp_clamp_collision_check': from_test(get_test_fn_clamp_clamp_collision_check(client, robot, process, options=options)),

            'attach_clamp_beam_collision_check': from_test(get_test_fn_clamp_beam_collision_check(client, robot, process, options=options)),
            'detach_clamp_beam_collision_check': from_test(get_test_fn_clamp_beam_collision_check(client, robot, process, options=options)),
        }