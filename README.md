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
python examples/drake_ompl_planning/main.py
```

This should move the arms from an initial configuration to a goal configuration using the RRTConnect planner.