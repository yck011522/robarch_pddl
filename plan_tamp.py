import os
import logging
import argparse
from termcolor import colored

import load_pddlstream

from pddlstream.algorithms.downward import set_cost_scale, parse_action, get_cost_scale
from pddlstream.utils import INF
from pddlstream.language.constants import print_plan, is_plan
from pddlstream.utils import flatten, Profiler, SEPARATOR, inf_generator, INF
from pddlstream.algorithms.meta import solve
from integral_timber_joints.planning.parsing import parse_process
# from integral_timber_joints.planning.pddlstream_planning.solve import solve_serialized_incremental

from utils import LOGGER, print_itj_pddl_plan
from parse import get_pddlstream_problem, PDDL_FOLDERS, DOMAIN_NAMES

##################################

def main():
    parser = argparse.ArgumentParser()
    # * Problem info
    parser.add_argument('--process', default='CantiBoxLeft_process.json', 
                        help='The name of the problem to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--design_dir', default='220407_CantiBoxLeft', 
                        help='problem json\'s containing folder\'s name.')

    # * Planning Problem Scope
    parser.add_argument('--disable_stream', action='store_true', help='Disable stream sampling in planning. Enable this will essentially ignore all the geometric constraints and all sampled predicate will be assumed always available. Defaults to False')
    parser.add_argument('--planning_case', metavar='N', type=int, default=1,
                        help='Which planning case to parse')
    parser.add_argument('--num_elements_to_export', metavar='N', type=int, default=-1,
                        help='Number of elements from process (may include skipped elements) to export. -1 means all elements are exported.')
    
    # * PDDLStream configs
    parser.add_argument('--costs', action='store_true', help='Use user-defined costs for actions.')
    # ! pyplanner config
    # parser.add_argument('--pp_h', default='ff', help='pyplanner heuristic configuration.')
    # parser.add_argument('--pp_search', default='eager', help='pyplanner search configuration.')
    # parser.add_argument('--pp_evaluator', default='greedy', help='pyplanner evaluator configuration.')
    # # ! downward config
    # parser.add_argument('--fd_search', default='ff-eager', help='downward search configuration.')

    # * Debugs, verbose and visuals
    parser.add_argument('--debug', action='store_true', help='Debug mode.')
    #
    args = parser.parse_args()
    LOGGER.info(f'Arguments: {args}')

    logging_level = logging.DEBUG if args.debug else logging.INFO
    LOGGER.setLevel(logging_level)

    #########
    # * Load process and convert to PDDLStream problem
    process = parse_process(args.design_dir, args.process) # , subdir=args.problem_subdir

    pddlstream_problem = get_pddlstream_problem(
        process = process,
        case_number = args.case_number,
        num_elements_to_export = args.num_elements_to_export,
        pddl_folder = PDDL_FOLDERS[args.case_number - 1],
        enable_stream = not args.disable_stream,
        )

    set_cost_scale(1)
    # with Profiler(num=25):
    if True:
        solution = solve(pddlstream_problem,
                         max_time=INF,
                         unit_costs=not args.costs,
                         success_cost=INF,
                        #  unit_efforts=True,
                        #  effort_weight=effort_weight,
                         max_planner_time=INF,
                         debug=args.debug, verbose=1, 
                        #  algorithm='incremental',
                        )

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