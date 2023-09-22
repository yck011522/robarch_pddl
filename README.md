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