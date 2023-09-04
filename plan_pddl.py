#!/usr/bin/env python

from __future__ import print_function
import argparse

import load_pddlstream
from pddlstream.algorithms.search import solve_from_pddl
from pddlstream.language.constants import get_length, read_pddl_pair

def dump_plan(plan, cost):
    solved = plan is not None
    print('Solved: {}'.format(solved))
    print('Cost: {}'.format(cost))
    print('Length: {}'.format(get_length(plan)))
    if not solved:
        return
    for i, action in enumerate(plan):
        print('{}) {}'.format(i+1, ' '.join(map(str, action))))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--problem', default='itj_gripper_only', 
                        help='The name of the problem to solve')
    args = parser.parse_args()

    domain_pddl, problem_pddl = read_pddl_pair(__file__, relative_dir=args.problem)
    plan, cost = solve_from_pddl(domain_pddl, problem_pddl)
    dump_plan(plan, cost)

if __name__ == '__main__':
    main()

