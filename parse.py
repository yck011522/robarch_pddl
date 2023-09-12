import os
import re
from termcolor import colored, cprint
from itertools import chain
from collections import defaultdict
import argparse

import load_pddlstream

from pddlstream.utils import read, write
from pddlstream.language.stream import DEBUG
from pddlstream.language.constants import And, Equal, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.language.temporal import parse_domain

from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.tools import Gripper, Screwdriver, Clamp
from integral_timber_joints.planning.utils import beam_ids_from_argparse_seq_n
from integral_timber_joints.planning.parsing import parse_process
from integral_timber_joints.assembly import BeamAssemblyMethod

from load_pddlstream import HERE
from utils import LOGGER
from write_pddl import pddl_problem_with_original_names

#################################################


def init_with_cost(manipulate_cost=5.0):
    init = [
        Equal(('Cost',), manipulate_cost),
        Equal((TOTAL_COST,), 0)
    ]
    return init


def extract_pddl_domain_name(pddl_folder):
    domain_pddl = read(os.path.join(pddl_folder, 'domain.pddl'))
    domain_name = parse_domain(domain_pddl).pddl
    return domain_name


def process_to_init_goal_beams(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        steps=-1,
        declare_static=False,
):

    # * All Beams
    for i, beam_id in enumerate(process.assembly.sequence):
        if (steps > -1) & (i >= steps):
            break
        #  Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue
        # Declare init and goal predicates
        init.extend([
            ('BeamAtStorage', beam_id),
        ])
        goal.extend([
            ('BeamAtAssembled', beam_id),
        ])
        # Declare static predicate of beam
        if declare_static:
            init.extend([
                ('Beam', beam_id),
            ])

    return init, goal


def process_to_init_goal_joints(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        steps=-1,
):

    # * Joints
    for i, beam_id in enumerate(process.assembly.sequence):
        if (steps > -1) & (i >= steps):
            break
        # Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue
        # Declare joints using already built neighbors (This will be compatiable with arbitary number of steps to export)
        for neighbor_id in process.assembly.get_already_built_neighbors(beam_id):
            init.extend([
                ('Joint', neighbor_id, beam_id),
            ])

    return init, goal


def process_to_init_goal_grippers(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        steps=-1,
        declare_static=False,
        replace_non_existent_gripper=True,
):
    """Convert a Process instance into a PDDLStream problem formulation
    """
    # * Gripper Tool
    available_gripper_types = []
    for gripper in process.grippers:
        # Declare init and goal predicates
        init.extend([
            ('GripperAtStorage', gripper.name),
            ('GripperOfType', gripper.name, gripper.type_name),
        ])
        goal.extend([
            ('GripperAtStorage', gripper.name),
        ])
        available_gripper_types.append(gripper.type_name)
        # Declare static predicate of gripper
        if declare_static:
            init.extend([
                ('Gripper', gripper.name),
            ])

    # * Beam Demand Gripper
    for i, beam_id in enumerate(process.assembly.sequence):
        if (steps > -1) & (i >= steps):
            break
        #  Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue
        gripper_type = process.assembly.get_beam_attribute(beam_id, "gripper_type")
        # Replace non existent gripper type with the first gripper type
        if gripper_type not in available_gripper_types:
            if replace_non_existent_gripper:
                gripper_type = available_gripper_types[0]
            else:
                raise ValueError(
                    f"Beam {beam_id} requires gripper type {gripper_type} but not available in the process.")
        init.extend([
            ('BeamNeedsGripperType', beam_id, gripper_type),
        ])

    return init, goal


def process_to_init_goal_assemblymethod(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        steps=-1,
):

    # * All Beams
    for i, beam_id in enumerate(process.assembly.sequence):
        if (steps > -1) & (i >= steps):
            break
        #  Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.CLAMPED:
            init.extend([('AssemblyByClampingMethod', beam_id),])
        if process.assembly.get_assembly_method(beam_id) in BeamAssemblyMethod.screw_methods:
            init.extend([('AssemblyByScrewingMethod', beam_id),])
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.GROUND_CONTACT:
            init.extend([('AssemblyByGroundConnection', beam_id),])
    return init, goal


def process_to_init_goal_clamps(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        steps=-1,
        declare_static=False,
):

    # * Clamp Tool
    for clamp in process.clamps:
        init.extend([
            ('ClampAtStorage', clamp.name),
            ('ClampOfType', clamp.name, clamp.type_name),
        ])
        goal.extend([
            ('ClampAtStorage', clamp.name),
        ])
        # Declare static predicate of clamp
        if declare_static:
            init.extend([
                ('Gripper', clamp.name),
            ])

    # * Beams Clamp type assignment
    for i, beam_id in enumerate(process.assembly.sequence):
        if (steps > -1) & (i >= steps):
            break
        #  Skip non clamped elements
        if process.assembly.get_assembly_method(beam_id) != BeamAssemblyMethod.CLAMPED:
            continue
        #  Iterate through the joints
        for neighbor_id in process.assembly.get_already_built_neighbors(beam_id):
            joint_clamp_type = process.assembly.get_joint_attribute(
                (neighbor_id, beam_id), 'tool_type')
            init.extend([
                ('JointNeedsClampType', neighbor_id, beam_id, joint_clamp_type),
            ])
    
    return init, goal


def process_to_init_goal_scaffolding(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        steps=-1,
        declare_static=False,
):

    # * Scaffolding
    last_beam_id = ""
    for i, beam_id in enumerate(process.assembly.sequence):
        if (steps > -1) & (i >= steps):
            break
        # BeamAssemblyMethod.MANUAL_ASSEMBLY are scaffolding
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            # Scaffolding elements will have a joint with previous element
            init.extend([
                ('ScaffoldingAtStorage', beam_id),
                ('BeamNeedsScaffolding', last_beam_id, beam_id),
            ])
            if declare_static:
                init.extend([
                    ('Scaffolding', beam_id),
                ])
        else:
            # Parse the beam id that the scaffolding belongs to:
            last_beam_id = beam_id

    return init, goal


# Utility functions for parsing

def get_pddlstream_problem(
        pddl_folder: str,
        process: RobotClampAssemblyProcess,
        enable_stream=True,
        **kwargs):
    """Convert a Process instance into a PDDLStream formulation
    """

    domain_pddl = read(os.path.join(pddl_folder, 'domain.pddl'))
    stream_pddl = read(os.path.join(HERE, pddl_folder, 'stream.pddl'))

    raise NotImplementedError(
        "get_pddlproblem_from_process() function needs to be rewritten")
    init, goal = get_pddlproblem_from_process(process, **kwargs)


    if not enable_stream:
        stream_map = DEBUG
    else:
        stream_map = {
            # 'sample-place_clamp_to_structure':  from_fn(get_action_ik_fn(client, robot, process, 'place_clamp_to_structure', options=options)),
        }

    constant_map = {}
    pddlstream_problem = PDDLProblem(
        domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return pddlstream_problem


def export_pddl(domain_name, init, goal, pddl_folder, problem_name):
    """export PDDL domain file
    """
    # parse domain pddl to make sure the domain and problem have consistent names

    # [parsed_domain_name] = re.findall(r'\(domain ([^ ]+)\)', domain_name)
    # problem_pddl_str = pddl_problem_with_original_names(problem_name, parsed_domain_name, init, goal)
    problem_pddl_str = pddl_problem_with_original_names(
        problem_name, domain_name, init, goal)

    pddl_problem_path = os.path.join(
        HERE, pddl_folder, 'problem_' + problem_name + '.pddl')
    write(pddl_problem_path, problem_pddl_str)
    LOGGER.info(colored('Exported PDDL domain file to {}'.format(
        pddl_problem_path), 'green'))


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    # Problem info (Input)
    parser.add_argument('--process', default='CantiBoxLeft_process.json',
                        help='The name of the process to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--design_dir', default='220407_CantiBoxLeft',
                        help='problem json\'s containing folder\'s name.')

    # Problem Info (Output)
    parser.add_argument('--planning_cases', metavar='N', type=int, nargs='+',
                        help='Which planning case to parse')
    parser.add_argument('--num_elements_to_export', metavar='N', type=int, default=-1,
                        help='Number of steps to export. -1 means all steps.')

    args = parser.parse_args()

    # Load process file
    # , subdir=args.problem_subdir
    process = parse_process(args.design_dir, args.process)
    process_name = os.path.splitext(os.path.basename(args.process))[0]
    problem_name = process_name

    steps = args.num_elements_to_export

    if 1 in args.planning_cases:
        #  Load domain.pddl
        pddl_folder = '01_beam_assembly'
        domain_name = 'beam_assembly'

        # Extract init and goal
        init, goal = process_to_init_goal_beams(process, steps=steps)
        unioned_goal = And(*goal)

        # Export PDDL domain file
        export_pddl(domain_name, init, unioned_goal, pddl_folder, problem_name)

    if 2 in args.planning_cases:
        #  Load domain.pddl
        pddl_folder = '02_joint_partial_order'
        domain_name = 'joint_partial_order'

        # Extract init and goal
        init, goal = process_to_init_goal_beams(process, steps=steps)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, steps=steps)
        unioned_goal = And(*goal)

        # Export PDDL domain file
        export_pddl(domain_name, init, unioned_goal, pddl_folder, problem_name)

    if 3 in args.planning_cases:
        #  Load domain.pddl
        pddl_folder = '03_gripper_switch'
        domain_name = 'gripper_switch'

        # Extract init and goal
        init, goal = process_to_init_goal_beams(process, steps=steps)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, steps=steps)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, steps=steps)
        unioned_goal = And(*goal)

        # Export PDDL domain file
        export_pddl(domain_name, init, unioned_goal, pddl_folder, problem_name)

    if 4 in args.planning_cases:
        #  Load domain.pddl
        pddl_folder = '04_assembly_stream'
        domain_name = 'assembly_stream'

        # Extract init and goal
        init, goal = process_to_init_goal_beams(
            process, steps=steps, declare_static=True)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, steps=steps)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, steps=steps)
        unioned_goal = And(*goal)

        # Export PDDL domain file
        export_pddl(domain_name, init, unioned_goal, pddl_folder, problem_name)

    if 5 in args.planning_cases:
        #  Load domain.pddl
        pddl_folder = '05_clamp_transfer'
        domain_name = 'clamp_transfer'

        # Extract init and goal
        init, goal = process_to_init_goal_beams(
            process, steps=steps, declare_static=True)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, steps=steps)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, steps=steps, declare_static=True)
        init, goal = process_to_init_goal_clamps(
            process, init, goal, steps=steps, declare_static=True)
        init, goal = process_to_init_goal_assemblymethod(process, init, goal, steps=steps)
        unioned_goal = And(*goal)

        # Export PDDL domain file
        export_pddl(domain_name, init, unioned_goal, pddl_folder, problem_name)

    if 6 in args.planning_cases:
        #  Load domain.pddl
        pddl_folder = '06_clamp_stream'
        domain_name = 'clamp_stream'

        # Extract init and goal
        init, goal = process_to_init_goal_beams(
            process, steps=steps, declare_static=True)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, steps=steps)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, steps=steps, declare_static=True)
        init, goal = process_to_init_goal_clamps(
            process, init, goal, steps=steps, declare_static=True)
        init, goal = process_to_init_goal_assemblymethod(process, init, goal, steps=steps)
        # init, goal = process_to_init_goal_scaffolding(process, init, goal, steps=steps, declare_static=True) # Probably not necessary
        unioned_goal = And(*goal)

        # Export PDDL domain file
        export_pddl(domain_name, init, unioned_goal, pddl_folder, problem_name)
