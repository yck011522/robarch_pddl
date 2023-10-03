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

from utils import LOGGER, print_itj_pddl_plan, save_plan_text, save_plan_dict
from parse_symbolic import PDDL_FOLDERS
from parse_tamp import get_pddlstream_problem

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
    parser.add_argument('--planning_cases', metavar='N', type=int, nargs='+', default=[4,6],
                        help='Which planning case to parse')
    parser.add_argument('--num_elements_to_export', metavar='N', type=int, default=-1,
                        help='Number of elements from process (may include skipped elements) to export. -1 means all elements are exported.')

    # * Output Config
    parser.add_argument('--output_to_file', action='store_true', help='Output plan to file.')
    parser.add_argument('--output_to_console', action='store_true', help='Output plan to console.')
    
    # * PDDLStream configs
    parser.add_argument('--costs', action='store_true', help='Use user-defined costs for actions.')
    # ! pyplanner config
    # parser.add_argument('--pp_h', default='ff', help='pyplanner heuristic configuration.')
    # parser.add_argument('--pp_search', default='eager', help='pyplanner search configuration.')
    # parser.add_argument('--pp_evaluator', default='greedy', help='pyplanner evaluator configuration.')
    # # ! downward config
    # parser.add_argument('--fd_search', default='ff-eager', help='downward search configuration.')

    # * Debugs, verbose and visuals
    parser.add_argument('--viewer', action='store_true', help='Enables the viewer during planning, default False')
    parser.add_argument('--debug', action='store_true', help='Debug mode.')
    parser.add_argument('--diagnosis', action='store_true', help='Diagnosis mode.')
    #
    args = parser.parse_args()
    LOGGER.info(f'Arguments: {args}')

    # logging_level = logging.DEBUG 
    logging_level = logging.DEBUG if args.debug else logging.INFO
    LOGGER.setLevel(logging_level)

    #########
    options = {
        'viewer' : args.viewer,
        # 'debug' : args.debug,
        'diagnosis' : args.diagnosis,
        # 'reinit_tool' : args.reinit_tool,
        'gantry_attempts' : 500, # number of gantry sampling attempts when computing IK
    }

    #########
    # * Load process and convert to PDDLStream problem
    process = parse_process(args.design_dir, args.process) # , subdir=args.problem_subdir

    for case_number in args.planning_cases:
        pddl_folder = PDDL_FOLDERS[case_number - 1]
        pddlstream_problem = get_pddlstream_problem(
            process = process,
            case_number = case_number,
            num_elements_to_export = args.num_elements_to_export,
            pddl_folder = pddl_folder,
            enable_stream = not args.disable_stream,
            options=options
            )

        set_cost_scale(1)
        # with Profiler(num=25):
        solution = solve(pddlstream_problem,
                         max_time=INF,
                         unit_costs=not args.costs,
                         success_cost=INF,
                        #  unit_efforts=True,
                        #  effort_weight=effort_weight,
                         max_planner_time=INF,
                        #  debug=args.debug, 
                         verbose=0, 
                        #  algorithm='incremental',
                        )

        plan, cost, evaluations = solution
        plan_success = is_plan(plan)

        #########
        # Print Results
        LOGGER.debug('-'*10)
        LOGGER.info(colored('Planning {}'.format('succeeds' if plan_success else 'fails'), 'green' if plan_success else 'red'))
        if is_plan(plan):
            # Output to Console
            if args.output_to_console:
                print_itj_pddl_plan(plan)

            # Output to File
            if args.output_to_file:
                with_stream = 'with_stream' if not args.disable_stream else 'without_stream'
                text_result_file_name = 'result_' + args.process + '_' + with_stream + '.txt'
                LOGGER.info('Plan result saved to {}.'.format(text_result_file_name))
                save_plan_text(plan, pddl_folder, text_result_file_name)

                # Save tamp result for visualization
                dict_result_file_name = 'result_' + args.process + '_' + with_stream + '.json'
                save_plan_dict(plan, pddl_folder, dict_result_file_name)
                LOGGER.info('Plan Json result saved to {}.'.format(dict_result_file_name))
        else:
            LOGGER.info('No plan found, no result saved.')

if __name__ == '__main__':
    main()