import os
from termcolor import cprint
from itertools import chain
from collections import defaultdict

import load_pddlstream
from load_pddlstream import HERE

from pddlstream.utils import read
from pddlstream.language.stream import StreamInfo, PartialInputs, WildOutput, DEBUG
from pddlstream.language.constants import And, PDDLProblem, Equal, print_plan, TOTAL_COST
from pddlstream.language.generator import from_gen_fn, from_fn, from_test

#################################################

def get_pddlstream_problem(problem: str,
        enable_stream=True, reset_to_home=True, 
        seq_n=None,
        use_fluents=True, 
        symbolic_only=False, 
        options=None):
    """Convert a Process instance into a PDDLStream formulation
    """
    options = options or {}

    domain_pddl = read(os.path.join(problem, 'domain.pddl'))
    # if not use_fluents:
    #     stream_pddl = read(os.path.join(HERE, problem, 'stream.pddl'))
    # else:
    #     stream_pddl = read(os.path.join(HERE, problem, 'stream_fluents.pddl'))

    manipulate_cost = 5.0
    init = [
        # Equal(('Cost',), manipulate_cost),
        # Equal((TOTAL_COST,), 0)
    ]
    # init.extend([
    #     ('RobotToolChangerEmpty',),
    # ])
    # home_conf = process.robot_initial_config
    constant_map = {}

    # * Beams
    n_beams = 2
    for i in range(n_beams):
        beam_name = f'beam{i+1}'
        init.extend([
            ('Beam', beam_name),
            ('BeamAtStorage', beam_name),
        ])

    # * Grippers
    n_grippers = 2
    for i in range(n_grippers):
        gripper_name = f'gripper{i+1}'
        init.extend([
            ('Gripper', gripper_name),
            ('GripperAtStorage', gripper_name),
        ])

    init.extend([
        ('GripperType', 'typeA'),
        ('GripperType', 'typeB'),
    ])

    # * Tool type matching
    init.extend([
        ('GripperOfType', 'gripper1', 'typeA'),
        ('GripperOfType', 'gripper2', 'typeB'),
    ])

    init.extend([
        ('BeamNeedsGripperType', 'beam1', 'typeA'),
        ('BeamNeedsGripperType', 'beam2', 'typeB'),
    ])

    if not enable_stream:
        stream_map = DEBUG
    else:
        stream_map = {
            # 'sample-place_clamp_to_structure':  from_fn(get_action_ik_fn(client, robot, process, 'place_clamp_to_structure', options=options)),
        }

    goal_literals = []
    goal_literals.extend(('BeamAtAssembled', f'beam{i+1}') for i in range(n_beams))
    if reset_to_home:
        goal_literals.extend([
            ('GripperAtStorage', f'gripper{i+1}') for i in range(n_grippers)
            ])
    goal = And(*goal_literals)

    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    return pddlstream_problem