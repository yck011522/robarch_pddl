# robarch_pddl
PDDL Experiments for RobArch Conference

## Installation

Run the following commands:
```
$ cd ext/pddlstream/downward
$ python ./build.py
```

Many development systems already satisfy FastDownward's dependencies. If `./build.py` fails, install FastDownward's dependencies using your package manager:
* APT (Linux): `$ sudo apt-get install cmake g++ g++-multilib make python`
* Windows:
    1. Install VS Studio Build Tools by downloading and running the executable from [here](https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=BuildTools&rel=16). Select the `C++ Build Tools` option in the installer.
    2. Install [CMake](https://cmake.org/) in your python environment by `pip install cmake`.
    3. Search for **Developer Command Prompt for VS 2017** (important!) in your search bar and run it.
    4. In the Developer Command Prompt for VS 2017, issue:
        ```
        cd /d "<path to FastDownward>"
        python ./build.py
        ```

If necessary, see FastDownward's documentation for more detailed installation instructions:

http://www.fast-downward.org/ObtainingAndRunningFastDownward

## Running the Experiments

### TAMP Cases (E01, E02, E03, E05)

```
python plan_pddl.py --planning_cases 1 --process_file_name CantiBoxLeft_process --symbolic_planner fd --debug --output_to_file 
```

It is also possible to run all the cases:
```
python plan_pddl.py --planning_cases 1 2 3 5 --process_file_name CantiBoxLeft_process CantiBoxMid_process CantiBoxRight_process --symbolic_planner fd --debug --output_to_file 
```
### PDDL Cases (E04, E06, E07)
The following code can be used to run the planning case one by one:
```
python plan_tamp.py --planning_case 4 --process CantiBoxLeft_process.json --design_dir 220407_CantiBoxLeft --debug --output_to_console --output_to_file --num_elements_to_export -1 
python plan_tamp.py --planning_case 4 --process CantiBoxMid_process.json --design_dir 220407_CantiBoxMid --debug --output_to_console --output_to_file --num_elements_to_export -1 
python plan_tamp.py --planning_case 4 --process CantiBoxRight_process.json --design_dir 220407_CantiBoxRight --debug --output_to_console --output_to_file --num_elements_to_export -1 
```

## Results
Note that the planning time will not be idendical everytime. The TAMP consis of a random search. The planning time is highly dependent on the speed of the computer. 

The planning time is written in the console after planning. For example the following planning took 112.67s in total. (run_time = sample_time + search_time)

```
Sampling for up to -95.222 seconds
Sampling while complexity <= 2
2023-10-12 17:32:45,507 | robarch_pddl | DEBUG | Cartesian plan sample found after 4 gantry iters.
2023-10-12 17:32:46,515 | robarch_pddl | DEBUG | Cartesian plan sample found after 0 gantry iters.
Summary: {complexity: 2, cost: 24.000, evaluations: 396, iterations: 7, length: 2, run_time: 112.670, sample_time: 107.825, search_time: 4.845, skeletons: 5, solutions: 1, solved: True, timeout: False}
```