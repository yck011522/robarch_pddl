import os

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
from parse_symbolic import process_to_init_goal_by_case
from stream_samplers import get_sample_fn_plan_motion_for_beam_assembly, get_test_fn_beam_assembly_collision_check

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
            'plan_motion_for_beam_assembly':  from_sampler(get_sample_fn_plan_motion_for_beam_assembly(client, robot, process, options=options)),
            'beam_assembly_collision_check': from_test(get_test_fn_beam_assembly_collision_check(client, robot, process, options=options)),
        }
    else:
        stream_map = DEBUG

    constant_map = {}
    pddlstream_problem = PDDLProblem(
        domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return pddlstream_problem

