# Course project for ECE1505 (Winter 2025) on convex optimization

This repository contains all the relevant scripts and tools needed to replicate the findings presented in our final project for the [ECE1505](https://www.comm.utoronto.ca/~weiyu/ece1505/) (Winter 2025) course on convex optimization. The LaTeX files for the project report can be found [here](docs/report/).

## Results
<img src="https://media.giphy.com/media/vFKqnCdLPNOKc/giphy.gif" width="250px" />

### GCS vs PRM: minimum-length trajectory
**Notes:**
- The best solutions obtained using GCS with 12 and 18 vertices are identical.
- Since both GCS (in these experiments) and PRM aim to find the shortest path, all resulting trajectories are traversed at a constant velocity.

<table>
  <tr>
    <th>GCS (6 vertices)</th>
    <th>GCS (12/18 vertices)</th>
    <th>PRM (no shortcutting)</th>
    <th>PRM (shortcutting)</th>
  </tr>
  <tr>
    <td><img src="figures/linear_6.png" width="250"/></td>
    <td><img src="figures/linear_12_18.png" width="250"/></td>
    <td><img src="figures/prm.png" width="244"/></td>
    <td><img src="figures/prm_w_shortcutting.png" width="244"/></td>
  </tr>
  <tr>
    <td><img src="figures/linear_6.gif" width="250"/></td>
    <td><img src="figures/linear_12_18.gif" width="250"/></td>
    <td><img src="figures/prm.gif" width="250"/></td>
    <td><img src="figures/prm_w_shortcutting.gif" width="250"/></td>
  </tr>
</table>

### GCS: extension
**Note:** A graph with 6 vertices was used in these experiments.
<table>
  <tr>
    <th width="300px">Time optmization</th>
    <th width="300px">Length optimization</th>
    <th width="300px">The energy of the time derivative of the trajectory optimization</th>
  </tr>
  <tr>
    <th width="300px"><img src="figures/bezier_100.png" width="250"/></th>
    <th width="300px"><img src="figures/bezier_010.png" width="250"/></th>
    <th width="300px"><img src="figures/bezier_001.png" width="250"/></th>
  </tr>
  <tr>
    <th width="300px"><img src="figures/bezier_100.gif" width="250"/></th>
    <th width="300px"><img src="figures/bezier_010.gif" width="250"/></th>
    <th width="300px"><img src="figures/bezier_001.gif" width="250"/></th>
  </tr>
</table>


## Installation

This project heavily depends on [Drake](https://drake.mit.edu) and the [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org). Follow the steps below to set up and install all necessary dependencies.

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

Navigate to the `./external/gcs` directory and install gcs dependencies by running:
```bash
pip install -r requirements.txt
```
**Note:** to successfully run our examples, you'll need to modify one line in the file `./external/gcs/gcs/linear.py`. Specifically, remove the `[1]` indexing from:
```bash
edge_length = edge.AddCost(Binding[Cost](
    self.edge_cost, np.append(u.x(), v.x())))[1]
```

### Setting up OMPL

This project depends on the latest 1.7.0 release of the Open Motion Planning Library (OMPL) and requires its Python bindings. As of this writing, OMPL does not have a PyPI package that can be installed directly via pip. Therefore, it must be installed manually. While there is no official installation method, the following is a recommended approach:

You can install the wheel file directly in your virtual environment. Download the appropriate wheel file from this [link](https://github.com/ompl/ompl/releases/tag/1.7.0) based on your system.

Once downloaded, install the wheel using `pip`:
```bash
pip install <wheel-file>.whl
```

## Usage

### Teleoperation of Manipulator Joints in Drake

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

The script will generate .pkl files under the benchmarks folder. You can use them to replicate the results in the report using the provided [notebook](notebooks/plot_statistics.ipynb).

Adjust the following configurations as needed:
```py
###############################
# Benchmarking Configurations #
###############################

PLANNERS = [
    "PRM",  # only PRM is supported at the moment
]

# this serves as a timeout value
# planning will stop as soon as it found an initial solution
RUNTIME_LIMIT = 3

RUN_COUNT = 1
SHORTCUTTING = True
PRECOMPUTE = False
PRECOMPUTATION_TIME = 10

# this is used when PRECOMPUTE is set to False

STORAGE_PATHS = {
    "PRM": os.path.join(
        CURRENT_DIR, "../data/PRM-600s.graph"
    ),  # 600s precomputed roadmap
}
```
`SHORTCUTTING` can be turned off to run the vanilla PRM; otherwise, it is enabled by default. We use the recent shortcutting approach proposed in [RRT-Rope](https://ieeexplore.ieee.org/abstract/document/9659071/?casa_token=litbc_-XX08AAAAA:03pWiotQEAXvHXJJBNOk5vNHlFvx1sWlrpQLe3qrlsBA7KdQYo1hQGBqj5sjU-d3hQgnVJBYEw). We also rely on a precomputed PRM roadmap to ensure consistent planning results, which is provided through the `STORAGE_PATHS` parameter. If you want to regenerate the roadmap, set the `PRECOMPUTE` parameter to True. This will generate a new PRM graph in the benchmarks folder—make sure to provide the correct `STORAGE_PATHS` for subsequent runs.

**Caveats**

We have experienced issues when loading the roadmap back into PRM using the OMPL planner's storage functionality. OMPL internally relies on Boost to serialize and deserialize storage data, and we suspect the issue may be due to mismatched Boost versions. This benchmark script has been successfully tested on macOS 15 but not on Ubuntu 22.04.

### Motion Planning with Graph of Convex Sets

We also formulate the problem of generating collision-free, kinodynamic motion plans as a mixed-integer convex program using the Graph of Convex Sets (GCS) approach. More information and runnable demos can be found [here](examples/drake_gcs_planning/).
