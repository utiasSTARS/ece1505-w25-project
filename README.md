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

> Only Python 3.8 is supported at the moment. There are some issues in manipulator-learning submodule that we need to fix before allowing the use of newer Python versions.

```bash
python3.8 -m venv .venv
source .venv/bin/activate
```

**Install Dependencies:**

Tkinter module for PyBullet GUI:
```bash
sudo apt install python3.8-tk
```

Main python dependencies:
```bash
pip install -r requirements.txt
```

## Usage