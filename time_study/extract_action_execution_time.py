import argparse
import os
import re

from collections import Counter
from datetime import datetime
from datetime import timedelta

from integral_timber_joints.planning.parsing import DESIGN_STUDY_DIR

from functions import *

# # Analysis Log Files generated by Process Controller

# Extracting the execution duration for each action and movements. There are two ways how the movement execution time is stored.
# 1. For all movments: extracting the logger timestamp at the begining and the end of the movement. 
# 2. For robotic movements: upon successful execution, a total time will be reported in a specific logging line.   

# General logging format: 
# ```['2022-06-13 08:42:01,347', 'app', 'INFO', 'App Started\n']```

def extract_action_time(design_dir, process_file_name, output_to_file):
    # Automatically grab all the Execution Log from the design directory
    
    log_folder = os.path.join(DESIGN_STUDY_DIR, design_dir, 'execution_log')
    log_file_names = [f for f in os.listdir(log_folder)]
    log_file_names = [f for f in log_file_names if f.startswith('ProcessExeController')]
    log_file_names = [f for f in log_file_names if f.endswith('.log')]

    # Read lines from the log files
    print('Found {} log files.'.format(len(log_file_names)))
    log_lines = []
    for log_file_name in log_file_names:
        print(' - {}'.format(log_file_name))
        file_path = os.path.join(log_folder, log_file_name)
        log_lines += load_log(file_path)

    num_lines = len(log_lines)
    print(f"Total Number of lines read from log file: {num_lines}")  # print number of lines

    # Apply filtering to remove invalid log file lines.
    valid_lines = filter_valid_log_lines(log_lines)
    num_valid_lines = len(valid_lines)  # get the number of lines read
    print(f"Number of valid lines read from log file: {num_valid_lines}")  # print number of lines

    # Load process file that contains all the planned actions and movements
    json_path = os.path.join(DESIGN_STUDY_DIR, design_dir, process_file_name)
    process = load_process(json_path)

    # Movement Duration from successfully executed Robotic Movement 
    # -----------------------------------------------------------

    # Extract lines that has a self-reported finishing time for the action.
    # Search for lines with a Movement ID and total time.


    pattern = r"\((A\d+_M\d+)\).* total time (.*) s"  # regular expression pattern to match (A1_M0) and the time

    robotic_movement_times = {}
    total_duration_s = timedelta()
    for time, app, scope, message in valid_lines:
        result = re.search(pattern, message)
        if result is not None:
            # print (message) 
            m_id = result.group(1)
            duration_s = float(result.group(2))
            robotic_movement_times[m_id] = duration_s
            total_duration_s += timedelta(seconds = duration_s)

    print(f"Total Number of movements: {len(robotic_movement_times)}")
    print(f"Total Duration = {total_duration_s} s")

    # Movement Duration from logger timestamp
    # -----------------------------------------------------------

    # Extract the begining time of the action as measured by the process controller.
    # Search for lines that starts with 'Executing' with a Movement ID in ().

    pattern_start = r"Executing .* \((A\d+_M\d+)\)"  # regular expression pattern to match (A1_M0) and the time
    pattern_end = r"(Next movement selected|Cannot select next movement)"  # regular expression pattern to match (A1_M0) and the time

    movement_time = []
    execution_duration = {}
    total_duration_s = timedelta()
    current_m_id = None
    current_m_start_time = None

    debug = False
    for time, app, scope, message in valid_lines:
        # Search for the start message
        result = re.search(pattern_start, message)
        if result is not None:
            # End previous movement
            if current_m_id is not None:
                duration = (time - current_m_start_time).total_seconds()
                execution_duration[current_m_id] = duration
                if debug: print ('End of {}, duration {}s (without end message)'.format(current_m_id, duration))
            # Start new movement
            current_m_id = result.group(1)
            current_m_start_time = time
            if debug: print ('Start of {} at {}'.format(current_m_id, current_m_start_time))
            
        # Search for the end message
        result = re.search(pattern_end, message)
        if result is not None:
            # End previous movement
            if current_m_id is not None:
                duration = (time - current_m_start_time).total_seconds()
                execution_duration[current_m_id] = duration
                if debug: print ('End of {}, duration {}s'.format(current_m_id, duration))
            current_m_id = None

    print(f"Total Number of movements: {len(execution_duration)}")
    print(f"Total Duration = {timedelta(seconds = sum(execution_duration.values()))} s")

    # Count all the actions and movements from the process file and determine their duration.
    # -----------------------------------------------------------
    # 1. Use values from robotic_movement_times if available
    # 2. Use values from execution_duration if available
    # 3. Replace values from similar Action-Movement pair average
    # 4. Use 0.0 as duration
    
    class_counts = {}
    movements_data_counter = Counter()

    # Data for all the actions
    process_time_data = {'actions' : []}
    for action in process.actions:
        # Data for one action and all the movements
        action_type = action.__class__.__name__
        act_n = action.act_n
        if debug: print('Action {} - {}'.format(act_n, action_type))
        action_time_data = {'movements' : [], 'action_type' : action_type, 'act_n' : act_n}
        process_time_data['actions'].append(action_time_data)

        for movement in action.movements:
            # Data for one movement
            movement_id = movement.movement_id
            movement_type = movement.__class__.__name__  # get the name of the object's class
            movement_time_data = {'movement_id' : movement_id, 'movement_type' : movement_type}
            action_time_data['movements'].append(movement_time_data)
            if movement_id in robotic_movement_times:
                # Use the time recorded in the robotic_movement_times, this is applicable to robotic movements.
                movement_time_data['duration'] = robotic_movement_times[movement_id]
            elif movement_id in execution_duration:
                # Otherwise use the time recorded by as execution_duration
                movement_time_data['duration'] = execution_duration[movement_id]
            else:
                # Otherwise the movement has no data and the total time of the action is not completely valid.
                movements_data_counter.update({'no_data' : 1})
            # Print movement data
            if debug: print('  Movement {} - {} - {}'.format(movement_id, movement_type, movement_time_data.get('duration', 'no data'))) 
        
        # Add the total time of the action
        action_total_time = sum([m['duration'] for m in action_time_data['movements'] if 'duration' in m])
        action_time_data['total_time'] = action_total_time

    # Total Process time
    process_total_time = sum([a['total_time'] for a in process_time_data['actions']])
    process_time_data['total_time'] = process_total_time
    print(f"Total Process Time: {timedelta(seconds = process_total_time)} s")

    # Filter data
    # For actions with missing movement time, an average based on other similar action-movement pair is used.
    # -----------------------------------------------------------

    # Organize the action / movement into keys where (action_type, i) represent the i-th movement of the action_type
    # Collect all the known durations for the movement and average them. 
    action_movement_durations = {}
    for actions in process_time_data['actions']:
        action_type = actions['action_type']
        for i, movement in enumerate(actions['movements']):
            if 'duration' in movement:
                key = (action_type, i) 
                if key not in action_movement_durations:
                    action_movement_durations[key] = []
                action_movement_durations[key].append(movement['duration'])

    # Average the durations
    average_action_movement_durations = {}
    for key, durations in action_movement_durations.items():
        average_action_movement_durations[key] = sum(durations) / len(durations)

    from copy import deepcopy
    data_with_patch_values = deepcopy(process_time_data)

    replacement_count = 0
    zero_value_count = 0
    for actions in data_with_patch_values['actions']:
        action_type = actions['action_type']
        for i, movement in enumerate(actions['movements']):
            key = (action_type, i)
            if 'duration' not in movement:
                if key in average_action_movement_durations:
                    actions['movements'][i]['duration'] = average_action_movement_durations[key]
                    replacement_count += 1
                else:
                    print('Missing data for {}, replaces with 0.0s '.format(key))
                    actions['movements'][i]['duration'] = 0.0
                    zero_value_count += 1

    print('Replaced {} missing values'.format(replacement_count))
    print('Replaced {} missing values with 0.0s'.format(zero_value_count))

    # data_with_patch_values can be used for subsequent estimation of the process time.
    # -----------------------------------------------------------

    # Accumulates the execution time by action type
    action_type_times = {}
    for actions in data_with_patch_values['actions']:
        action_type = actions['action_type']
        if action_type not in action_type_times:
            action_type_times[action_type] = []
        action_type_times[action_type].append(actions['total_time'])

    # Compute the average time needed for each action type
    print('Aerage time needed for each action type: ')
    itj_action_average_time = {}
    for action_type, times in action_type_times.items():
        itj_action_average_time[action_type] = sum(times) / len(times)
        print ('- {} \t  average time = {:.2f}s (count={})'.format(action_type, sum(times) / len(times), len(times)))

    # Compute pddl action average time
    # -----------------------------------------------------------
    # A translation is necessary between pddl actions and integral_timber_joints actions
    
    pddl_actions_itj_actions = {
    'assemble_beam_by_clamping_method' : [
        'LoadBeamAction',
        'PickBeamWithGripperAction',
        'BeamPlacementWithClampsAction',
        'RetractGripperFromBeamAction',
        ],
    'assemble_beam_by_screwing_method' : [
        'LoadBeamAction',
        'PickScrewdriverFromStorageAction',
        'CloseGripperOnBeamAction',
        'GenericGripperApproachBeamPickupAction',
        'PickAndRotateBeamForAttachingScrewdriverAction',
        'OperatorAttachScrewdriverAction',
        'AssembleBeamWithScrewdriversAction',
        'RetractScrewdriverFromBeamAction',
        'PlaceScrewdriverToStorageAction'
        ],
    'assemble_beam_by_ground_connection' : [
        'LoadBeamAction',
        'BeamPlacementWithoutClampsAction',
        ],
    'pick_gripper_from_storage' : ['PickGripperFromStorageAction'],
    'place_gripper_to_storage' : ['PlaceGripperToStorageAction'],
    'pick_clamp_from_storage' : ['PickClampFromStorageAction'],
    'place_clamp_to_storage' : ['PlaceClampToStorageAction'],
    'attach_clamp_to_joint ' : ['PlaceClampToStructureAction'],
    'detach_clamp_from_joint' : ['PickClampFromStructureAction'],
    }

    print ('PDDL Action Average Time : ')

    pddl_action_average_times = {}
    for pddl_action , itj_actions in pddl_actions_itj_actions.items():
        pddl_action_time = 0.0
        for itj_action in itj_actions:
            if itj_action in itj_action_average_time:
                pddl_action_time += itj_action_average_time[itj_action]
        pddl_action_average_times[pddl_action] = pddl_action_time
        print ('- {} \t  average time = {:.2f}s'.format(pddl_action, pddl_action_time))
    
    if output_to_file:
        import json
        process_file_name_no_ext = os.path.splitext(process_file_name)[0]
        output_file_path = os.path.join('time_study', process_file_name_no_ext + '_time.json')
        with open(output_file_path, 'w') as f:
            dict = {'pddl_action_average_times' : pddl_action_average_times, 'itj_action_average_time' : itj_action_average_time}
            json.dump(dict, f, indent=4)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Problem info (Input)
    parser.add_argument('--process_file_names', metavar='N', type=str, nargs='+', 
                        default=['CantiBoxLeft_process.json', 'CantiBoxMid_process.json', 'CantiBoxRight_process.json'],
                        help='The name of the process to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--design_dirs', metavar='N', type=str, nargs='+', 
                        default=['220407_CantiBoxLeft', '220407_CantiBoxMid', '220407_CantiBoxRight'],
                        help='problem json\'s containing folder\'s name.')

    # Planning Problem Scope
    parser.add_argument('--planning_cases', metavar='N', type=int, nargs='+',
                        help='Which planning case to parse')
    # Output Config
    parser.add_argument('--output_to_file', action='store_true', help='Output plan to file.')

    args = parser.parse_args()

    # Parse execution duration from log files
    # Values of the three different problem files are separated
    # -----------------------------------------------------------
    design_dirs = args.design_dirs
    process_file_names = args.process_file_names
    assert len(design_dirs) == len(process_file_names)
    output_to_file = args.output_to_file

    for design_dir, process_file_name in zip(design_dirs, process_file_names):
        print('Processing {} in {}'.format(process_file_name, design_dir))
        extract_action_time(design_dir, process_file_name, output_to_file)


