import load_pddlstream
from collections import OrderedDict, Sequence
from pddlstream.language.constants import EQ, CONNECTIVES, QUANTIFIERS, OBJECTIVES, \
    get_prefix, get_args, is_parameter, Fact, Equal
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.algorithms.common import add_fact, INIT_EVALUATION
from pddlstream.language.conversion import is_atom, is_negated_atom, objects_from_evaluations

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

def replace_expression(parent, fn):
    prefix = get_prefix(parent)
    if prefix == EQ:
        assert(len(parent) == 3)
        value = parent[2]
        if isinstance(parent[2], Sequence):
            value = replace_expression(value, fn)
        return prefix, replace_expression(parent[1], fn), value
    elif prefix in (CONNECTIVES + OBJECTIVES):
        children = parent[1:]
        return (prefix,) + tuple(replace_expression(child, fn) for child in children)
    elif prefix in QUANTIFIERS:
        assert(len(parent) == 3)
        parameters = parent[1]
        child = parent[2]
        return prefix, parameters, replace_expression(child, fn)
    # name = get_prefix(parent).lower()
    name = get_prefix(parent)
    args = get_args(parent)
    return Fact(name, map(fn, args))

def obj_from_value_expression(parent):
    return replace_expression(parent, lambda o: o if is_parameter(o) else Object.from_value(o))

def evaluations_from_init(init):
    evaluations = OrderedDict()
    for raw_fact in init:
        fact = obj_from_value_expression(raw_fact)
        add_fact(evaluations, fact, result=INIT_EVALUATION, complexity=0)
    return evaluations

def pddl_problem_with_original_names(domain_name, init, goal):
    problem_name = domain_name
    evaluations = evaluations_from_init(init)
    goal_expression = obj_from_value_expression(goal)
    
    objects = objects_from_evaluations(evaluations)
    s = '(define (problem {})\n' \
           '\t(:domain {})\n' \
           '\t(:objects {})\n' \
           '\t(:init \n\t\t{})\n' \
           '\t(:goal {})'.format(
        problem_name, 
        domain_name,
        ' '.join(sorted(list(map(pddl_from_object_og, objects)))),
        '\n\t\t'.join(sorted(filter(lambda p: p is not None,
                                    map(pddl_from_evaluation_og, evaluations)))),
        pddl_from_expression_og(goal_expression))

    # if objective is not None:
    #     s += '\n\t(:metric minimize ({}))'.format(objective)

    return s + ')\n'