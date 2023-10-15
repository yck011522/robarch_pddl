import argparse
import json
import os

from integral_timber_joints.planning.parsing import DESIGN_STUDY_DIR
from integral_timber_joints.assembly import BeamAssemblyMethod
from log_parse_functions import load_process

# Load the pddl_action_average_times from json file
def load_pddl_action_average_times(filepath):
    with open(filepath) as json_file:
        pddl_action_average_times = json.load(json_file)
    return pddl_action_average_times 

def extract_pddl_actions_from_result_json(result_file_name):

    with open(result_file_name, 'r') as f:
        from compas.data import DataDecoder
        # Deserialize asert correctness and add to Process
        tamp_result = json.load(f, cls=DataDecoder) #type: list

    pddl_actions = []
    for sequence in tamp_result:
        pddl_actions_in_result = sequence['actions']
        for pddl_action in pddl_actions_in_result:
            action_name = pddl_action['action_name']
            if action_name == 'assemble_beam_by_clamping_method':
                pddl_actions.append('assemble_beam_by_clamping_method')
            if action_name == 'assemble_beam_by_screwing_method':
                pddl_actions.append('assemble_beam_by_screwing_method')
            if action_name == 'assemble_beam_by_ground_connection':
                pddl_actions.append('assemble_beam_by_ground_connection')
            if action_name == 'pick_gripper_from_storage':
                pddl_actions.append('pick_gripper_from_storage')
            if action_name == 'place_gripper_to_storage':
                pddl_actions.append('place_gripper_to_storage')
            if action_name == 'clamp_from_storage_to_joint':
                pddl_actions.append('pick_gripper_from_storage')
                pddl_actions.append('attach_clamp_to_joint')
            if action_name == 'clamp_from_joint_to_storage':
                pddl_actions.append('detach_clamp_from_joint')
                pddl_actions.append('place_clamp_to_storage')
            if action_name == 'clamp_from_joint_to_joint':
                pddl_actions.append('detach_clamp_from_joint')
                pddl_actions.append('attach_clamp_to_joint')

    return pddl_actions

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    # Problem info (Input)

    output_to_file = True
    folder_name = '06_clamp_stream'
    result_file_names = [
        r'result_CantiBoxLeft_process.json_with_stream.json',
        r'result_CantiBoxMid_process.json_with_stream.json',
        r'result_CantiBoxRight_process.json_with_stream.json',
    ]
    output_file_names = [
        r'result_CantiBoxLeft_process.json_with_stream.json',
        r'result_CantiBoxMid_process.json_with_stream.json',
        r'result_CantiBoxRight_process.json_with_stream.json',
    ]

    for result_file_name, output_file_name in zip(result_file_names,output_file_names):

        # Flowchart action planning 
        pddl_actions = extract_pddl_actions_from_result_json(os.path.join(folder_name,result_file_name))


        # Load the pddl_action_average_times from json file
        process_file_name_no_ext = result_file_name.split('.')[0].split('result_')[1]
        average_time_file_path = os.path.join('time_study', process_file_name_no_ext + '_time.json')
        pddl_action_average_times = load_pddl_action_average_times(average_time_file_path)

        # PDDL actions are grouped by types in the paper
        from action_group import PDDL_ACTIONS_TYPE_GROUPS
        action_type_groups = PDDL_ACTIONS_TYPE_GROUPS

        # Calculate the estimated time for each action type
        estimated_times = {}
        for action_type in action_type_groups:
            estimated_times[action_type] = {'count' : 0, 'total_time_s' : 0.0, 'total_time_min' : 0.0}

        for action in pddl_actions:
            for action_type in action_type_groups:
                if action in action_type_groups[action_type]:
                    estimated_times[action_type]['count'] += 1
                    time = pddl_action_average_times['pddl_action_average_times'][action]
                    estimated_times[action_type]['total_time_s'] += time
                    estimated_times[action_type]['total_time_min'] += time / 60.0

        # Calculate the estimated time for overall actions
        estimated_times['overall_total_time_s'] = sum([estimated_times[action_type]['total_time_s'] for action_type in action_type_groups])
        estimated_times['overall_total_time_min'] = sum ([estimated_times[action_type]['total_time_min'] for action_type in action_type_groups])
        
        # Display results and save to file
        print (estimated_times)
        if output_to_file:
            output_file_path = os.path.join('time_study', process_file_name_no_ext + '_case_6' + '_estimated_time.json')
            with open(output_file_path, 'w') as f:
                json.dump(estimated_times, f, indent=4)