from datetime import datetime
from integral_timber_joints.process import *
import json
from compas.utilities import DataDecoder
import numpy as np


def load_log(file_path):
    log_file = open(file_path, 'r')  # open the log file in read mode
    log_lines = []  # create an empty list to store the split lines

    for line in log_file:
        split_line = line.split(" - ")  # split each line using " - "
        if len(split_line) == 4:
            log_lines.append(split_line)  # add the split line to the list

    log_file.close()  # close the log file

    num_lines = len(log_lines)  # get the number of lines read
    print(f"Number of lines read from log file: {num_lines}")  # print number of lines
    return log_lines

def filter_valid_log_lines(log_lines):
    num_lines = len(log_lines)
    valid_lines = []
    for i in range(num_lines):
        try:
            date_string = log_lines[i][0]
            datetime_obj = datetime.strptime(date_string, "%Y-%m-%d %H:%M:%S,%f")
            valid_line = [datetime_obj] + log_lines[i][1:]
            # print (valid_line)
            valid_lines.append(valid_line)
        except ValueError:
            pass
    return valid_lines

def filter_dict_none_values(dict):
    for key in list(dict.keys()):
        # If the value of the key is None, delete the key-value pair
        if dict[key] is None:
            del dict[key]

def load_process(json_path):
    process = None
    with open(json_path, 'r') as f:
        process = json.load(f, cls=DataDecoder)

    print("Process Loaded (%i Beams, %i Actions, %i Movements)" % (
                    len(process.assembly.sequence),
                    len(process.actions),
                    len(process.movements)))
    return process


def mean_percentile(data, lower_bound, upper_bound):

    lower = np.percentile(data, lower_bound)  # calculate the lower bound of the data
    upper = np.percentile(data, upper_bound)  # calculate the upper bound of the data
    print (lower, upper)
    filtered_data = list(filter(lambda x: lower <= x <= upper, data))  # filter out values outside the bounds
    mean = np.mean(filtered_data)  # calculate the mean of filtered_data

    return mean