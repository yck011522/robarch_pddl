#!/usr/bin/env python

from __future__ import print_function
import argparse
from termcolor import colored

import load_pddlstream
from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.language.constants import get_length, read_relative_dir, is_plan
from pddlstream.algorithms.downward import parse_sequential_domain, parse_problem, task_from_domain_problem
from pddlstream.algorithms.instantiate_task import sas_from_pddl, instantiate_task, convert_instantiated
from pddlstream.language.attachments import solve_pyplanners
from pddlstream.utils import Verbose

from utils import LOGGER, print_itj_pddl_plan, save_plan_text, save_plan_dict
from parse_symbolic import PDDL_FOLDERS, DOMAIN_NAMES, HERE

def dump_plan(plan, cost):
    solved = plan is not None
    LOGGER.info('Solved: {}'.format(solved))
    LOGGER.info('Cost: {}'.format(cost))
    LOGGER.info('Length: {}'.format(get_length(plan)))
    if not solved:
        return
    for i, action in enumerate(plan):
        LOGGER.info('{}) {}'.format(i+1, ' '.join(map(str, action))))

def solve_pddl_symbolic(pddl_folder, pddl_problem_name, symbolic_planner_name, debug=False):
    domain_pddl, problem_pddl = read_relative_dir(__file__, filenames=['domain.pddl', pddl_problem_name], relative_dir=pddl_folder)
    
    # Calling FastDownward Planner
    if symbolic_planner_name == 'fd':
        use_greedy_planner = True
        planner = 'ff-astar2' if use_greedy_planner else 'ff-astar'
        plan, cost = solve_from_pddl(domain_pddl, problem_pddl, debug=debug, clean=True, max_planner_time=600, planner = planner)
    
    # Calling Pyplanners Planner
    elif symbolic_planner_name == 'pyplanners':
        domain = parse_sequential_domain(domain_pddl)
        problem = parse_problem(domain, problem_pddl)
        task = task_from_domain_problem(domain, problem, add_identical=False)
        with Verbose(debug):
            instantiated = instantiate_task(task)
            instantiated = convert_instantiated(instantiated)
        plan, cost =  solve_pyplanners(instantiated)

    return plan, cost

def main():
    parser = argparse.ArgumentParser()

    # Planning Problem
    parser.add_argument('--planning_cases', metavar='N', type=int, nargs='+', default=[1,2,3,5],
                        help='Which planning case to parse [1,2,3,5]')
    parser.add_argument('--process_file_names', nargs='+', default=['CantiBoxLeft_process'], 
                        help='problem pddl file name')
    # Planner Config 
    parser.add_argument('--symbolic_planners', nargs='+', default=['pyplanners', 'fd'])
    parser.add_argument('--debug', action='store_true')
    # Output Config
    parser.add_argument('--output_to_file', action='store_true', help='Output plan to file.')
    parser.add_argument('--output_to_console', action='store_true', help='Output plan to console.')

    args = parser.parse_args()
    
    for symbolic_planner_name in args.symbolic_planners:
        for case_number in args.planning_cases:
            for process_file_name in args.process_file_names:
                # Parse Arguments
                if case_number not in [1,2,3,5]:
                    raise NotImplementedError('Only planning case 1,2,3,5 are supported.')
                pddl_folder = PDDL_FOLDERS[case_number - 1]
                pddl_problem_name = 'problem_' + process_file_name + '.pddl'

                debug=args.debug

                # Call Planner
                plan, cost = solve_pddl_symbolic(pddl_folder, pddl_problem_name, symbolic_planner_name, debug=debug)

                # Print Results
                if is_plan(plan):
                    LOGGER.info(colored('CaseFolder = {}, \tproblem = {}. \tPlan found. Cost: {}'.format(pddl_folder, pddl_problem_name, cost),'green'))
                    # dump_plan(plan, cost)
                else:
                    LOGGER.info(colored('CaseFolder = {}, \tproblem = {}. \tPlan not found.'.format(pddl_folder, pddl_problem_name),'red'))
                    continue
                
                # Output to Console
                if args.output_to_console:
                    print_itj_pddl_plan(plan)

                # Output to File
                if args.output_to_file:
                    if is_plan(plan):
                        # Save text excerpt
                        text_result_file_name = 'result_' + process_file_name + '_' + symbolic_planner_name + '.txt'
                        save_plan_text(plan, pddl_folder, text_result_file_name)
                        LOGGER.info('Plan Text result saved to {}.'.format(text_result_file_name))
                        
                        # Save tamp result for visualization
                        dict_result_file_name = 'result_' + process_file_name + '_' + symbolic_planner_name + '.json'
                        save_plan_dict(plan, pddl_folder, dict_result_file_name)
                        LOGGER.info('Plan Json result saved to {}.'.format(dict_result_file_name))
                    else:
                        LOGGER.info('No plan found, no result saved.')

if __name__ == '__main__':
    main()

