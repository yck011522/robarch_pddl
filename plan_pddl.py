#!/usr/bin/env python

from __future__ import print_function
import argparse

import load_pddlstream
from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.language.constants import get_length, read_relative_dir
from utils import LOGGER, print_itj_pddl_plan

def dump_plan(plan, cost):
    solved = plan is not None
    LOGGER.info('Solved: {}'.format(solved))
    LOGGER.info('Cost: {}'.format(cost))
    LOGGER.info('Length: {}'.format(get_length(plan)))
    if not solved:
        return
    for i, action in enumerate(plan):
        LOGGER.info('{}) {}'.format(i+1, ' '.join(map(str, action))))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pddl_folder', default='itj_clamps', 
                        help='The folder of the pddl domain and problem')
    parser.add_argument('--pddl_problem', default='problem.pddl', 
                        help='problem pddl file name')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    LOGGER.info(f'Arguments: {args}')

    domain_pddl, problem_pddl = read_relative_dir(__file__, filenames=['domain.pddl', args.pddl_problem], relative_dir=args.pddl_folder)
    plan, cost = solve_from_pddl(domain_pddl, problem_pddl, debug=args.debug, clean=True)

    # dump_plan(plan, cost)
    print_itj_pddl_plan(plan)

if __name__ == '__main__':
    main()

