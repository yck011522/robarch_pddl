import os
import logging
import argparse
from termcolor import colored
from functools import partial

import load_pddlstream

from pddlstream.algorithms.downward import set_cost_scale, parse_action, get_cost_scale
from pddlstream.utils import INF
from pddlstream.language.constants import print_plan, is_plan
from pddlstream.utils import flatten, Profiler, SEPARATOR, inf_generator, INF
from pddlstream.algorithms.meta import solve
# from integral_timber_joints.planning.pddlstream_planning.solve import solve_serialized_incremental

from utils import LOGGER
from parse import get_pddlstream_problem

from pddlstream.utils import str_from_object
from pddlstream.language.conversion import obj_from_pddl
from pddlstream.language.constants import is_plan, DurativeAction, Action, StreamAction, FunctionAction

##################################

def contains_number(value):
    for character in value:
        if character.isdigit():
            return True
    return False

def colored_str_from_object(obj, show_details=False):
    if not show_details:
        # if isinstance(obj, Frame):
        #     return '(frm)'
        # elif isinstance(obj, Transformation):
        #     return '(tf)'
        # elif isinstance(obj, Configuration):
        #     return colored('(conf)', 'yellow')
        if isinstance(obj, Action):
            return colored(obj, 'yellow')

    str_rep = str_from_object(obj)
    if contains_number(str_rep):
        return colored(str_rep, 'blue')
    else:
        return colored(str_rep, 'red')

def print_itj_pddl_plan(plan, show_details=False):
    if not is_plan(plan):
        return
    step = 1
    color_print_fn = partial(colored_str_from_object, show_details=show_details)
    for action in plan:
        if isinstance(action, DurativeAction):
            name, args, start, duration = action
            LOGGER.info('{:.2f} - {:.2f}) {} {}'.format(start, start+duration, name,
                                                  ' '.join(map(str_from_object, args))))
        elif isinstance(action, Action):
            name, args = action
            LOGGER.info('{:2}) {} {}'.format(step, colored(name, 'green'), ' '.join(map(color_print_fn, args))))
            step += 1
        elif isinstance(action, StreamAction):
            name, inputs, outputs = action
            LOGGER.info('    {}({})->({})'.format(name, ', '.join(map(str_from_object, inputs)),
                                            ', '.join(map(str_from_object, outputs))))
        elif isinstance(action, FunctionAction):
            name, inputs = action
            LOGGER.info('    {}({})'.format(name, ', '.join(map(str_from_object, inputs))))
        else:
            raise NotImplementedError(action)

##################################

def main():
    parser = argparse.ArgumentParser()
    # * Problem info
    parser.add_argument('--problem', default='itj_gripper_only', 
                        help='The name of the problem to solve')
    #
    # * PDDLStream configs
    parser.add_argument('--nofluents', action='store_true', help='Not use fluent facts in stream definitions.')
    # parser.add_argument('--symbolics', action='store_true', help='Use the symbolic-only PDDL formulation.')
    # parser.add_argument('--disable_stream', action='store_true', help='Disable stream sampling in planning. Enable this will essentially ignore all the geometric constraints and all sampled predicate will be assumed always available. Defaults to False')
    parser.add_argument('--no_return_rack', action='store_true', help='Add all-tools-back-to-rack to the goal.')
    parser.add_argument('--costs', action='store_true', help='Use user-defined costs for actions.')
    # ! pyplanner config
    parser.add_argument('--pp_h', default='ff', help='pyplanner heuristic configuration.')
    parser.add_argument('--pp_search', default='eager', help='pyplanner search configuration.')
    parser.add_argument('--pp_evaluator', default='greedy', help='pyplanner evaluator configuration.')
    # ! downward config
    parser.add_argument('--fd_search', default='ff-eager', help='downward search configuration.')

    # * Planning for sub-assembly
    # parser.add_argument('--seq_n', nargs='+', type=int, help='Zero-based index according to the Beam sequence in process.assembly.sequence. If only provide one number, `--seq_n 1`, we will only plan for one beam. If provide two numbers, `--seq_n start_id end_id`, we will plan from #start_id UNTIL #end_id. If more numbers are provided. By default, all the beams will be checked.')

    # * Debugs, verbose and visuals
    parser.add_argument('--debug', action='store_true', help='Debug mode.')
    #
    args = parser.parse_args()
    LOGGER.info(f'Arguments: {args}')

    logging_level = logging.DEBUG if args.debug else logging.INFO
    LOGGER.setLevel(logging_level)

    #########
    # * PDDLStream problem conversion and planning
    LOGGER.info(colored('Using {} backend.'.format('pyplanner' if not args.nofluents else 'downward'), 'cyan'))
    pddlstream_problem = get_pddlstream_problem(args.problem,
        enable_stream= False, #not args.disable_stream, 
        reset_to_home=not args.no_return_rack, 
        use_fluents=not args.nofluents, 
        # seq_n=args.seq_n, 
        # symbolic_only=args.symbolics
        )

    if args.debug:
        print_pddl_task_object_names(pddlstream_problem)

    additional_config = {}
    if not args.nofluents:
        additional_config['planner'] = {
            'search': args.pp_search, # eager | lazy | hill_climbing | a_star | random_walk | mcts
            'evaluator': args.pp_evaluator, # 'bfs' | 'uniform' | 'astar' | 'wastar2' | 'wastar3' | 'greedy'
            'heuristic': args.pp_h, # goal | add | ff | max | blind #'heuristic': ['ff', get_bias_fn(element_from_index)],
            'successors': 'all', # all | random | first_goals | first_operators # 'successors': order_fn,
        }
    else:
        # https://github.com/caelan/pddlstream/blob/4914667a13a80831cadaf115a70938e9f93b021e/pddlstream/algorithms/downward.py#L87
        additional_config['planner'] = args.fd_search
        # 'dijkstra' # 'max-astar' # 'lmcut-astar' # 'dijkstra' # 'ff-eager' # | 'add-random-lazy'

    set_cost_scale(1)
    # effort_weight = 1. / get_cost_scale()
    # with Profiler(num=25):
    if True:
        solution = solve(pddlstream_problem,
                         max_time=INF,
                         unit_costs=not args.costs,
                         success_cost=INF,
                        #  unit_efforts=True,
                        #  effort_weight=effort_weight,
                         max_planner_time=INF,
                         debug=args.debug, verbose=1, **additional_config)
        # if reset_to_home
        # solution = solve_serialized_incremental(pddlstream_problem,
        #                  max_time=INF,
        #                  unit_costs=not args.costs,
        #                  success_cost=INF,
        #                  max_planner_time=INF,
        #                  debug=args.debug, verbose=1, **additional_config)

    plan, cost, evaluations = solution
    plan_success = is_plan(plan)

    #########
    # * PDDLStream problem conversion and planning
    LOGGER.debug('-'*10)
    print_itj_pddl_plan(plan)
    LOGGER.info(colored('Planning {}'.format('succeeds' if plan_success else 'fails'), 'green' if plan_success else 'red'))

    # if plan_success:
    #     LOGGER.info(f'Plan length: {len(plan)}')
    #     if plan_success and args.write:
    #         # save_pddlstream_plan_to_itj_process(process, plan, args.design_dir, args.problem, verbose=1, save_subdir=args.save_dir)

    #         log_file_path = os.path.join(os.path.dirname(get_process_path(args.design_dir, args.problem, args.save_dir)), os.path.basename(args.problem).split('.')[0] + '.log')
    #         process.debug_print_process_actions_movements(log_file_path)
    #         LOGGER.info(f"Action Log saved to: {log_file_path}")

if __name__ == '__main__':
    main()