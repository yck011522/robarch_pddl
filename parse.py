from operator import is_
import os
import re
from termcolor import colored, cprint
from itertools import chain
from collections import defaultdict

from pddlstream.utils import read, write
from pddlstream.language.stream import DEBUG
from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.language.write_pddl import get_problem_pddl
from pddlstream.algorithms.downward import write_pddl
from pddlstream.language.constants import And, PDDLProblem, Equal, TOTAL_COST
from pddlstream.language.generator import from_gen_fn, from_fn, from_test

from integral_timber_joints.process import RobotClampAssemblyProcess
from integral_timber_joints.planning.utils import beam_ids_from_argparse_seq_n

import load_pddlstream
from load_pddlstream import HERE
from utils import LOGGER

#################################################

def get_pddlstream_problem(
        pddl_domain_name: str,
        process : RobotClampAssemblyProcess,
        enable_stream=True, 
        reset_to_home=True, 
        seq_n=None,
        use_fluents=True, 
        symbolic_only=False, 
        options=None):
    """Convert a Process instance into a PDDLStream formulation
    """
    options = options or {}

    domain_pddl = read(os.path.join(pddl_domain_name, 'domain.pddl'))
    if not use_fluents:
        stream_pddl = read(os.path.join(HERE, pddl_domain_name, 'stream.pddl'))
    else:
        stream_pddl = read(os.path.join(HERE, pddl_domain_name, 'stream_fluents.pddl'))

    process_symdata = process.to_symbolic_problem_data()

    manipulate_cost = 5.0
    init = [
        Equal(('Cost',), manipulate_cost),
        Equal((TOTAL_COST,), 0)
    ]

    # * Beams
    beam_seq = beam_ids_from_argparse_seq_n(process, seq_n)
    for i, e in enumerate(beam_seq):
        beam_data = process_symdata['assembly']['sequence'][i]
        assert beam_data['beam_id'] == e
        assert beam_data['assembly_method'] != 'UNDEFINED'
        LOGGER.debug('{} : {}'.format(e, beam_data['assembly_method']+'Element'))

        beam_gripper_type = beam_data["beam_gripper_type"]
        if beam_data['assembly_method'] == 'ManualAssembly':
            pass
            # init.extend([
            #     ('Scaffold', e),
            #     ])
        else:
            init.extend([
                ('Beam', e),
                ('BeamAtStorage', e),
                ('BeamNeedsGripperType', e, beam_gripper_type),
                ])
            # init.append((e_data['assembly_method']+'Element', e))
            # for sf in e_data['associated_scaffolds']:
            #     init.append(('AssociatedScaffold', e, sf))

    # init.append(('FirstElement', beam_seq[0]))

    # * joint to clamp/scewdriver tool type assignment
    joints_data = [j_data for j_data in process_symdata['assembly']['joints'] if j_data['joint_id'][0] in beam_seq and \
            j_data['joint_id'][1] in beam_seq]
    for j_data in joints_data:
        j = j_data['joint_id']
        joint_clamp_type = j_data['tool_type']
        init.extend([
            ('Joint', j[0], j[1]),
            ('JointNeedsClamp', j[0], j[1], joint_clamp_type),
        ])

    # * assembly sequence
    # cprint('Using beam sequence ordering: {}'.format(beam_seq), 'yellow')
    # for e1, e2 in zip(beam_seq[:-1], beam_seq[1:]):
    #     init.append(('Order', e1, e2))

    # * Grippers
    for g_name, g_data in process_symdata['grippers'].items():
        # g = process.gripper(g_name)
        init.extend([
            ('Gripper', g_name),
            ('GripperAtStorage', g_name),
            ('GripperOfType', g_name, g_data['type_name']),
        ])

    # * Clamps
    for c_name, c_data in process_symdata['clamps'].items():
        # c = process.clamp(c_name)
        init.extend([
            ('Clamp', c_name),
            ('ClampAtStorage', c_name),
            ('ClampOfType', c_name, c_data['type_name']),
        ])

    # * Screw Drivers
    # if 'screwdrivers' in process_symdata:
    #     for sd_name in process_symdata['screwdrivers']:
    #         sd = process.screwdriver(sd_name)
    #         init.extend([
    #             ('ScrewDriver', sd_name),
    #             ('AtRack', sd_name),
    #         ])

    if not enable_stream:
        stream_map = DEBUG
    else:
        stream_map = {
            # 'sample-place_clamp_to_structure':  from_fn(get_action_ik_fn(client, robot, process, 'place_clamp_to_structure', options=options)),
            # 'sample-pick_clamp_from_structure':  from_fn(get_action_ik_fn(client, robot, process, 'pick_clamp_from_structure', options=options)),
        }

    goal_literals = []
    goal_literals.extend(('BeamAtAssembled', e) for e in beam_seq)
    if reset_to_home:
        goal_literals.extend(('GripperAtStorage', t_name) for t_name in process_symdata['grippers'])
        goal_literals.extend(('ClampAtStorage', t_name) for t_name in process_symdata['clamps'])
    goal = And(*goal_literals)

    constant_map = {}
    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    # * export PDDL domain file
    evaluations, goal_expression, domain, _ = parse_problem(pddlstream_problem)
    # problem_pddl = get_problem_pddl(evaluations, goal_expression, domain.pddl, temporal=False)
    problem_pddl = pddl_problem_with_original_names(domain, evaluations, goal_expression)

    pddl_problem_path = os.path.join(HERE, pddl_domain_name, 'exported_domain.pddl')
    write(pddl_problem_path, problem_pddl)
    LOGGER.info(colored('Exported PDDL domain file to {}'.format(pddl_problem_path), 'green'))

    return pddlstream_problem

from pddlstream.language.conversion import obj_from_pddl, is_atom, is_negated_atom
from pddlstream.language.conversion import pddl_from_object, objects_from_evaluations
from pddlstream.language.object import Object, OptimisticObject

def pddl_from_object_og(obj):
    if isinstance(obj, str):
        return obj
    return obj.value
    
def pddl_head_og(name, args):
    return '({})'.format(' '.join([name] + list(map(pddl_from_object_og, args))))

def pddl_from_evaluation_og(evaluation):
    head = pddl_head_og(evaluation.head.function, evaluation.head.args)
    if is_atom(evaluation):
        return head
    elif is_negated_atom(evaluation):
        return '(not {})'.format(head)
    value = evaluation.value
    return '(= {} {})'.format(head, value)

def pddl_from_expression_og(expression):
    if isinstance(expression, Object) or isinstance(expression, OptimisticObject):
        return pddl_from_object_og(expression)
    if isinstance(expression, str):
        return expression
    return '({})'.format(' '.join(map(pddl_from_expression_og, expression)))

def pddl_problem_with_original_names(domain, evaluations, goal_expression, objective=None):
    [domain_name] = re.findall(r'\(domain ([^ ]+)\)', domain.pddl)
    problem_name = domain_name
    
    objects = objects_from_evaluations(evaluations)
    s = '(define (problem {})\n' \
           '\t(:domain {})\n' \
           '\t(:objects {})\n' \
           '\t(:init \n\t\t{})\n' \
           '\t(:goal {})'.format(
        problem_name, 
        domain_name,
        ' '.join(list(map(pddl_from_object_og, objects))),
        '\n\t\t'.join(sorted(filter(lambda p: p is not None,
                                    map(pddl_from_evaluation_og, evaluations)))),
        pddl_from_expression_og(goal_expression))
    if objective is not None:
        s += '\n\t(:metric minimize ({}))'.format(objective)
    return s + ')\n'

