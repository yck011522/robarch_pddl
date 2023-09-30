import logging
import os
from load_pddlstream import HERE
from termcolor import colored
from functools import partial

import load_pddlstream
from pddlstream.utils import str_from_object
from pddlstream.language.conversion import obj_from_pddl
from pddlstream.language.constants import is_plan, DurativeAction, Action, StreamAction, FunctionAction

###########################################
# borrowed from: https://github.com/compas-dev/compas_fab/blob/3efe608c07dc5b08653ee4132a780a3be9fb93af/src/compas_fab/backends/pybullet/utils.py#L83
def get_logger(name):
    logger = logging.getLogger(name)

    try:
        from colorlog import ColoredFormatter
        formatter = ColoredFormatter("%(log_color)s%(levelname)-8s%(reset)s %(white)s%(message)s",
                                     datefmt=None,
                                     reset=True,
                                     log_colors={'DEBUG': 'cyan', 'INFO': 'green',
                                                 'WARNING': 'yellow',
                                                 'ERROR': 'red', 'CRITICAL': 'red',
                                                 }
                                     )
    except ImportError:
        formatter = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')

    formatter = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    return logger

LOGGER = get_logger('robarch_pddl')

###########################################

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

def pddl_plan_to_string(plan):
    plan_string_lines = []
    step = 1
    for action in plan:
        if isinstance(action, DurativeAction):
            name, args, start, duration = action
            plan_string_lines.append('{:.2f} - {:.2f}) {} {}'.format(start, start+duration, name,
                                                  ' '.join(map(str_from_object, args))))
        elif isinstance(action, Action):
            name, args = action
            plan_string_lines.append('{:3}: {} {}'.format(step, name, ' '.join(map(str_from_object, args))))
            step += 1
        elif isinstance(action, StreamAction):
            name, inputs, outputs = action
            plan_string_lines.append('    {}({})->({})'.format(name, ', '.join(map(str_from_object, inputs)),
                                            ', '.join(map(str_from_object, outputs))))
        elif isinstance(action, FunctionAction):
            name, inputs = action
            plan_string_lines.append('    {}({})'.format(name, ', '.join(map(str_from_object, inputs))))
        else:
            raise NotImplementedError(action)
    return plan_string_lines

def pddl_plan_to_dict(plan):
    seq_n = 0 # Increment after the assembly of each beam
    act_n = 0 # Increment after every action , resets after new beam
    sequences = []
    
    sequence = {'seq_n': seq_n, 'actions': []}
    for action in plan:
        if isinstance(action, Action):
            action_name, args = action
            sequence['actions'].append({'act_n': act_n, 'action_name': action_name, 'args': args})
            act_n += 1
            if action_name.startswith('assemble_beam_'):
                seq_n += 1
                act_n = 0
                sequence['beam_id'] = args[0]
                sequences.append(sequence)
                sequence = {'seq_n': seq_n, 'actions': []}
    return sequences

def save_plan_text(plan, pddl_folder, file_name):  
    # Create folder if not exists
    if not os.path.exists(pddl_folder):
        os.makedirs(pddl_folder)

    # Save plan to file
    file_output_path = os.path.join(HERE, pddl_folder, file_name)
    with open(file_output_path, 'w') as f:
        for line in pddl_plan_to_string(plan):
            f.write(line + '\n')


def save_plan_dict(plan, pddl_folder, file_name):
    # Create folder if not exists
    if not os.path.exists(pddl_folder):
        os.makedirs(pddl_folder)

    # Save plan to file
    file_output_path = os.path.join(HERE, pddl_folder, file_name)
    action_dict = pddl_plan_to_dict(plan)
    import json
    with open(file_output_path, 'w') as f:
        json.dump(action_dict, f, indent=4)

