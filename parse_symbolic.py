import os
from termcolor import colored
import argparse

import load_pddlstream

from pddlstream.utils import read, write
from pddlstream.language.constants import And, Equal, TOTAL_COST
from pddlstream.language.temporal import parse_domain

from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.tools import Gripper, Screwdriver, Clamp
from integral_timber_joints.planning.utils import beam_ids_from_argparse_seq_n
from integral_timber_joints.planning.parsing import parse_process
from integral_timber_joints.assembly import BeamAssemblyMethod

from load_pddlstream import HERE
from utils import LOGGER
from export_pddl_utils import pddl_problem_with_original_names

#################################################


PDDL_FOLDERS = ['01_beam_assembly', '02_joint_partial_order', '03_gripper_switch',
                '04_assembly_stream', '05_clamp_transfer', '06_clamp_stream']
DOMAIN_NAMES = ['beam_assembly', 'joint_partial_order',
                'gripper_switch', 'assembly_stream', 'clamp_transfer', 'clamp_stream']


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
        num_elements_to_export=-1,
        declare_static=False,
):

    # * All Beams
    for i, beam_id in enumerate(process.assembly.sequence):
        if (num_elements_to_export > -1) & (i >= num_elements_to_export):
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
        num_elements_to_export=-1,
):

    # * Joints
    for i, beam_id in enumerate(process.assembly.sequence):
        if (num_elements_to_export > -1) & (i >= num_elements_to_export):
            break
        # Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue
        # Declare joints using already built neighbors (This will be compatiable with arbitary number of num_elements_to_export to export)
        for neighbor_id in process.assembly.get_already_built_neighbors(beam_id):
            init.extend([
                ('Joint', neighbor_id, beam_id),
            ])

    return init, goal


def process_to_init_goal_grippers(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        num_elements_to_export=-1,
        declare_static=False,
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
        if (num_elements_to_export > -1) & (i >= num_elements_to_export):
            break
        #  Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue
        gripper_type = process.assembly.get_beam_attribute(
            beam_id, "gripper_type")

        # * Declare screwdriver if this beam is a ScrewedWithoutGripper type and this screwdriver has not been declared
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER and \
            gripper_type not in available_gripper_types:
                for screwdriver in process.screwdrivers:
                    if screwdriver.type_name != gripper_type:
                        continue
                    # Declare init and goal predicates
                    init.extend([
                        ('GripperAtStorage', screwdriver.name),
                        ('GripperOfType', screwdriver.name, screwdriver.type_name),
                    ])
                    goal.extend([
                        ('GripperAtStorage', screwdriver.name),
                    ])
                    available_gripper_types.append(screwdriver.type_name)
                    # Declare static predicate of gripper
                    if declare_static:
                        init.extend([
                            ('Gripper', screwdriver.name),
                        ])

        if gripper_type not in available_gripper_types:
            raise ValueError(
                f"Beam {beam_id} requires gripper type {gripper_type} but not available in the process.")

        init.extend([
            ('BeamNeedsGripperType', beam_id, gripper_type),
        ])

    return init, goal


def process_to_init_goal_assemblymethod(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        num_elements_to_export=-1,
):

    # * All Beams
    for i, beam_id in enumerate(process.assembly.sequence):
        if (num_elements_to_export > -1) & (i >= num_elements_to_export):
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
        num_elements_to_export=-1,
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
                ('Clamp', clamp.name),
            ])

    # * Beams Clamp type assignment
    for i, beam_id in enumerate(process.assembly.sequence):
        if (num_elements_to_export > -1) & (i >= num_elements_to_export):
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
        num_elements_to_export=-1,
        declare_static=False,
):

    # * Scaffolding
    last_beam_id = ""
    for i, beam_id in enumerate(process.assembly.sequence):
        if (num_elements_to_export > -1) & (i >= num_elements_to_export):
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


def process_to_init_goal_fixed_assembly_order(
        process: RobotClampAssemblyProcess,
        init=[], goal=[],
        num_elements_to_export=-1,
):
    for i, (beam_id1, beam_id2) in enumerate(zip(process.assembly.sequence[:-1], process.assembly.sequence[1:])):
        if (num_elements_to_export > -1) & (i >= num_elements_to_export):
            break
        #  Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id1) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue
        init.append(
            ('AssemblyPartialOrder', beam_id1, beam_id2)
            )

    return init, goal


# Utility functions for parsing

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


def process_to_init_goal_by_case(
        process: RobotClampAssemblyProcess,
        case_number: int,
        init=[], goal=[],
        num_elements_to_export=-1
):
    """Case Number represent the the six planning cases defined in the paper. 
    Steps : the number of beams in process.assembly_sequence to export. Default -1 means all num_elements_to_export.
    """
    # Extract init and goal
    if case_number == 1:
        init, goal = process_to_init_goal_beams(
            process, num_elements_to_export=num_elements_to_export)

    if case_number == 2:
        init, goal = process_to_init_goal_beams(
            process, num_elements_to_export=num_elements_to_export)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, num_elements_to_export=num_elements_to_export)

    if case_number == 3:
        init, goal = process_to_init_goal_beams(
            process, num_elements_to_export=num_elements_to_export)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, num_elements_to_export=num_elements_to_export)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, num_elements_to_export=num_elements_to_export)

    if case_number == 4:
        init, goal = process_to_init_goal_beams(
            process, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, num_elements_to_export=num_elements_to_export)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, num_elements_to_export=num_elements_to_export)

    if case_number == 5:
        init, goal = process_to_init_goal_beams(
            process, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, num_elements_to_export=num_elements_to_export)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_clamps(
            process, init, goal, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_assemblymethod(
            process, init, goal, num_elements_to_export=num_elements_to_export)

    if case_number == 6:
        init, goal = process_to_init_goal_beams(
            process, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, num_elements_to_export=num_elements_to_export)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_clamps(
            process, init, goal, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_assemblymethod(
            process, init, goal, num_elements_to_export=num_elements_to_export)
        # init, goal = process_to_init_goal_scaffolding(process, init, goal, num_elements_to_export=num_elements_to_export, declare_static=True) # Probably not necessary

    if case_number == 7:
        init, goal = process_to_init_goal_beams(
            process, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_joints(
            process,  init, goal, num_elements_to_export=num_elements_to_export)
        init, goal = process_to_init_goal_grippers(
            process, init, goal, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_clamps(
            process, init, goal, num_elements_to_export=num_elements_to_export, declare_static=True)
        init, goal = process_to_init_goal_assemblymethod(
            process, init, goal, num_elements_to_export=num_elements_to_export)
        # init, goal = process_to_init_goal_scaffolding(process, init, goal, num_elements_to_export=num_elements_to_export, declare_static=True) # Probably not necessary
        init, goal = process_to_init_goal_fixed_assembly_order(process, init, goal, num_elements_to_export=num_elements_to_export)

    unioned_goal = And(*goal)
    return init, unioned_goal


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Problem info (Input)
    parser.add_argument('--process_file_name', default='CantiBoxLeft_process.json',
                        help='The name of the process to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--design_dir', default='220407_CantiBoxLeft',
                        help='problem json\'s containing folder\'s name.')

    # Planning Problem Scope
    parser.add_argument('--planning_cases', metavar='N', type=int, nargs='+',
                        help='Which planning case to parse')
    parser.add_argument('--num_elements_to_export', metavar='N', type=int, default=-1,
                        help='Number of elements from process (may include skipped elements) to export. -1 means all elements are exported.')

    args = parser.parse_args()

    # Load process file
    process_file_name = args.process_file_name
    process = parse_process(args.design_dir, process_file_name)
    process_name = os.path.splitext(os.path.basename(process_file_name))[0]
    problem_name = process_name

    num_elements_to_export = args.num_elements_to_export

    # Hard coded domain names and folder names
    pddl_folders = PDDL_FOLDERS
    domain_names = DOMAIN_NAMES

    # Create PDDL problem from process and export
    for case_number in range(1, 7):
        if case_number in args.planning_cases:
            # Extract init and goal
            init, unioned_goal = process_to_init_goal_by_case(
                process, case_number, [], [], num_elements_to_export=num_elements_to_export)
            # Export PDDL domain file
            export_pddl(domain_names[case_number - 1], init,
                        unioned_goal, pddl_folders[case_number - 1], problem_name)
