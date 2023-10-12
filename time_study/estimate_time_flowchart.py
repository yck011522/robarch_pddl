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

def simulated_flowchart_action_planning(process):
    # Flowchart action planning 
    pddl_actions = []
    for beam_id in process.assembly.sequence:
        beam_assembly_method = process.assembly.get_assembly_method(beam_id)
        if beam_assembly_method == BeamAssemblyMethod.GROUND_CONTACT:
            pddl_actions.append('pick_gripper_from_storage')
            pddl_actions.append('assemble_beam_by_ground_connection')
            pddl_actions.append('place_gripper_to_storage')

        elif beam_assembly_method == BeamAssemblyMethod.CLAMPED:
            number_of_clamped_joints = len(process.assembly.get_joints_of_beam_connected_to_already_built(beam_id))
            for i in range(number_of_clamped_joints):
                pddl_actions.append('pick_clamp_from_storage')
                pddl_actions.append('attach_clamp_to_joint')
            pddl_actions.append('pick_gripper_from_storage')
            pddl_actions.append('assemble_beam_by_clamping_method')
            pddl_actions.append('place_gripper_to_storage')
            for i in range(number_of_clamped_joints):
                pddl_actions.append('detach_clamp_from_joint')
                pddl_actions.append('place_clamp_to_storage')

        elif beam_assembly_method == BeamAssemblyMethod.SCREWED_WITH_GRIPPER or beam_assembly_method == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER:
            pddl_actions.append('pick_gripper_from_storage')
            pddl_actions.append('assemble_beam_by_screwing_method')
            pddl_actions.append('place_gripper_to_storage')

    return pddl_actions

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    # Problem info (Input)
    parser.add_argument('--process_file_names', metavar='N', type=str, nargs='+', 
                        default=['CantiBoxLeft_process.json', 'CantiBoxMid_process.json', 'CantiBoxRight_process.json'],
                        help='The name of the process to solve (json file\'s name, e.g. "nine_pieces_process.json")')
    parser.add_argument('--design_dirs', metavar='N', type=str, nargs='+', 
                        default=['220407_CantiBoxLeft', '220407_CantiBoxMid', '220407_CantiBoxRight'],
                        help='problem json\'s containing folder\'s name.')
    parser.add_argument('--output_to_file', action='store_true', help='Output plan to file.')

    args = parser.parse_args()

    design_dirs = args.design_dirs
    process_file_names = args.process_file_names
    assert len(design_dirs) == len(process_file_names)
    output_to_file = args.output_to_file

    for design_dir, process_file_name in zip(design_dirs, process_file_names):
        # Load process file that contains all the planned actions and movements
        json_path = os.path.join(DESIGN_STUDY_DIR, design_dir, process_file_name)
        process = load_process(json_path)

        # Flowchart action planning 
        pddl_actions = simulated_flowchart_action_planning(process)

        # Load the pddl_action_average_times from json file
        process_file_name_no_ext = os.path.splitext(process_file_name)[0]
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
            output_file_path = os.path.join('time_study', process_file_name_no_ext + '_flowchart' + '_estimated_time.json')
            with open(output_file_path, 'w') as f:
                json.dump(estimated_times, f, indent=4)