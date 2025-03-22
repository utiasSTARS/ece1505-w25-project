# Course project for ECE1505 (Winter 2025) on convex optimization

## Installation

**Clone the Repository (including submodules):**

```bash
git clone --recurse-submodules https://github.com/utiasSTARS/ece1505-w25-project
cd ece1505-w25-project
```

If you forgot `--recurse-submodules`, run these commands inside the repo:
```bash
git submodule init
git submodule update
```

**Create and Activate a Virtual Environment:**

It is highly recommended to use a virtual environment to isolate project dependencies:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
```

**Install Dependencies:**

Main python dependencies:
```bash
pip install -r requirements.txt
```

### Setting up OMPL

This project requires the Python bindings for OMPL. As of this writing, OMPL does not have a PyPI package that can be installed directly via pip. Therefore, it must be installed manually. While there is no official installation method, the following is a recommended approach:

You can install the wheel file directly in your virtual environment. Download the appropriate wheel file from this [link](https://github.com/ompl/ompl/releases/tag/prerelease) based on your system. OMPL is expected to release version 1.7.0 soon, but for now, we will use the 1.6.0 pre-release version.

Once downloaded, install the wheel using `pip`:
```bash
pip install <wheel-file>.whl
```

Note:

If you set up OMPL as described above, you can use most of the planners available in the upstream OMPL repository. However, our benchmarking script also uses [Greedy RRT*](https://arxiv.org/abs/2405.03411v2), a variant of RRTConnect that can find solutions as quickly as RRTConnect but with anytime properties. To use this planner, you need to use [my OMPL fork](https://github.com/mlsdpk/ompl/tree/greedyrrtstar) and build OMPL and its bindings from source.


## Usage

Run the following command in the top-level directory of the repo:
```bash
python examples/joints_teleop.py --initial_conf <initial_conf>
```
<initial_conf> can be zeros, start or goal.
After running the command, click on the provided link (likely http://localhost:7000). 
Once the page opens in your browser, click the "Open Controls" button 
in the top-right corner to open the joint control sliders.

### Running Drake + OMPL demo
Run the following command in the top-level directory of the repository:
```bash
python examples/drake_ompl_planning.py
```

This should move the arms from an initial configuration to a goal configuration using the RRTConnect planner.

### Benchmarking OMPL Planners

You can run the following script to benchmark the performance of OMPL planners:
```bash
python examples/drake_ompl_benchmarking.py
```

Adjust the following configurations as needed:
```py
###############################
# Benchmarking Configurations #
###############################

PLANNERS = [
    "RRT",
    "EST",
    "RRTConnect",
    "RRTstar",
    "PRMstar",
    "FMTstar",
    "InformedRRTstar",
    "BITstar",
    # "GreedyRRTstar",
]

RUNTIME_LIMIT = 10
MEMORY_LIMIT = 4096
RUN_COUNT = 10
```

This script will generate several files under the benchmarks folder of the project, organized by timestamp. You can use the `.db` file to analyze the results using [Planner Arena](http://plannerarena.org/). More information about the database format and log files can be found [here](https://ompl.kavrakilab.org/benchmark.html). You can also run Planner Arena locally - instructions can be found [here](https://ompl.kavrakilab.org/plannerarena.html).

#### Precompute and Reuse Roadmaps

This feature is not well-documented or widely promoted in OMPL, so not many people are aware of it. Here's a brief explanation of how it works, along with some examples of how to achieve it.

TODO (Phone): update this section with more details.

Another related topic is Experience-based Planning. OMPL has older [Thunder](https://ompl.kavrakilab.org/classompl_1_1tools_1_1Thunder.html) and [Lightning](https://ompl.kavrakilab.org/classompl_1_1tools_1_1Lightning.html) frameworks, which use retrieve-and-repair paradigms. We are not using it in this project, so I won’t cover it here — but will come back at some point.