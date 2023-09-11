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

def get_pddlproblem_from_process(process: RobotClampAssemblyProcess, steps = -1, export_joint_tools = True, export_scaffolding = True):
    """Convert a Process instance into a PDDLStream problem formulation
    """

    # manipulate_cost = 5.0
    init = [
        # Equal(('Cost',), manipulate_cost),
        # Equal((TOTAL_COST,), 0)
    ]
    goal = []

    # * Beams
    for i, beam_id in enumerate(process.assembly.sequence):
        if steps > -1 & i >= steps:
            break
        #  Required Gripper Type
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            # Scaffolding elements will have a joint with previous element
            if export_scaffolding:
                init.extend([
                    ('BeamScaffolding', process.assembly.sequence[i-1], beam_id),
                    ('BeamAtStorage', beam_id),
                    ])
        else:
            init.extend([
                ('Beam', beam_id),
                ('BeamNeedsGripperType', beam_id, process.assembly.get_beam_attribute(beam_id, "gripper_type")),
                ('BeamAtStorage', beam_id),
                ])
            goal.extend([
                ('BeamAtAssembled', beam_id),
                ])

    # * Grippers
    for gripper in process.grippers:
        init.extend([
            ('GripperAtStorage', gripper.name),
            ('GripperOfType', gripper.name, gripper.type_name),
        ])
        goal.extend([
            ('GripperAtStorage', gripper.name),
        ])

    # * Joint declaration and clamp/scewdriver tool type assignment
    for i, beam_id in enumerate(process.assembly.sequence):
        if steps > -1 & i >= steps:
            break
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.CLAMPED:
            for neighbor_id in process.assembly.get_already_built_neighbors(beam_id):
                joint_clamp_type = process.assembly.get_joint_attribute((neighbor_id, beam_id), 'tool_type')
                init.extend([
                    ('Joint', neighbor_id, beam_id),
                    ])
                if export_joint_tools:
                    init.extend([
                        ('JointNeedsClamp', neighbor_id, beam_id, joint_clamp_type),
                    ])
        if process.assembly.get_assembly_method(beam_id) in [BeamAssemblyMethod.SCREWED_WITH_GRIPPER,BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER]:
            for neighbor_id in process.assembly.get_already_built_neighbors(beam_id):
                init.extend([
                    ('Joint', neighbor_id, beam_id),
                    ])
                if export_joint_tools:
                    init.extend([
                        ('JointNeedsScrewdriver', neighbor_id, beam_id),
                    ])

    # * Clamps
    if export_joint_tools:
        for clamp in process.clamps:
            init.extend([
                ('ClampAtStorage', clamp.name),
                ('ClampOfType', clamp.name, clamp.type_name),
            ])
            goal.extend([
                ('ClampAtStorage', clamp.name),
            ])

    unioned_goal = And(*goal)
    return init, unioned_goal

def get_pddlstream_problem(
        pddl_folder: str,
        process : RobotClampAssemblyProcess,
        enable_stream=True, 
        **kwargs):
    """Convert a Process instance into a PDDLStream formulation
    """

    domain_pddl = read(os.path.join(pddl_folder, 'domain.pddl'))
    stream_pddl = read(os.path.join(HERE, pddl_folder, 'stream.pddl'))

    init, goal = get_pddlproblem_from_process(process, **kwargs)

    if not enable_stream:
        stream_map = DEBUG
    else:
        stream_map = {
            # 'sample-place_clamp_to_structure':  from_fn(get_action_ik_fn(client, robot, process, 'place_clamp_to_structure', options=options)),
        }

    constant_map = {}
    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    return pddlstream_problem

def export_pddl(domain_name, init, goal, pddl_folder, problem_name):
    """export PDDL domain file
    """
    # parse domain pddl to make sure the domain and problem have consistent names
    
    [parsed_domain_name] = re.findall(r'\(domain ([^ ]+)\)', domain_name)
    problem_pddl_str = pddl_problem_with_original_names(problem_name, parsed_domain_name, init, goal)

    pddl_problem_path = os.path.join(HERE, pddl_folder, 'problem_' + problem_name + '.pddl')
    write(pddl_problem_path, problem_pddl_str)
    LOGGER.info(colored('Exported PDDL domain file to {}'.format(pddl_problem_path), 'green'))

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    # Problem info (Input)
    parser.add_argument('--process', default='CantiBoxLeft_process.json', 
    # parser.add_argument('--process', default='pavilion_process.json', 
                        # CantiBoxLeft_10pcs_process.json
                        help='The name of the process to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--design_dir', default='220407_CantiBoxLeft', 
    # parser.add_argument('--design_dir', default='210128_RemodelFredPavilion', 
                        # 210916_SymbolicPlanning
                        help='problem json\'s containing folder\'s name.')
    # Problem Info (Output)
    parser.add_argument('--pddl_folder', default='itj_gripper_only', # itj_clamps_fromprocess
                        help='The folder of the pddl problem to solve')

    # Problem simplication
    parser.add_argument('--no_scaffolding', action='store_true',
                        help='do not export scaffolding info to the pddl problem.')
    parser.add_argument('--no_joint_tools', action='store_true',
                        help='do not export clamp info to the pddl problem.')
    args = parser.parse_args()

    # Load domain.pddl
    domain_pddl = read(os.path.join(args.pddl_folder, 'domain.pddl'))

    # Load process file
    process = parse_process(args.design_dir, args.process) # , subdir=args.problem_subdir

    # Naming
    process_name = os.path.splitext(os.path.basename(args.process))[0]
    domain_name = parse_domain(domain_pddl).pddl
    problem_name = process_name

    # Extract init and goal
    init, goal = get_pddlproblem_from_process(process, export_joint_tools = not args.no_joint_tools, export_scaffolding = not args.no_scaffolding)

    # Export PDDL domain file
    export_pddl(domain_name, init, goal, args.pddl_folder, problem_name)
