# Quasi-Steady-State Black Box Simulation for the Generation of g-g-g-v Diagrams

# Introduction
The g-g diagram visualizes the maximum longitudinal
and lateral accelerations that a vehicle can achieve. To address non-planar
road geometries, this concept can be extended to incorporate
g-g constraints as a function of vehicle speed and vertical
acceleration, commonly referred to as g-g-g-v diagrams. This repository provides a parallelized black box simulation toolchain to generate QSS g-g-g-v diagram. Our approach enables the use of existing high-fidelity vehicle models without the need for differentiability, avoiding model mismatch problems.

<img src="./src/graphics/gggv_tornado.png" alt="Tornado" width="500"/>

## Basic Functionality
The core functionality involves a fully factorial batch execution of steering ramp maneuvers across specified ranges of velocity (`v`), longitudinal acceleration (`ax`), and vertical acceleration (`gt`).

In the simulation, the vehicle maintains a quasi-steady-state at the target speed through external forces applied to its center of gravity, emulating inertial forces via the targeted longitudinal acceleration. A drive torque controller compensates for resistance forces to sustain the target speed.

<img src="./src/graphics/vehicle_sketch.png" alt="Vehicle Sketch" width="500"/>

## Simulation Process
Each simulation starts with a small steering step. The resulting lateral acceleration jump calculates an appropriate steering ramp speed, achieving a controlled lateral acceleration rise rate essential for quasi-static driving conditions. The maximum lateral acceleration recorded in each simulation becomes the result.

- **Stable Runs:** The maximum recorded lateral acceleration is used as the result.
- **Unstable Runs:** If instability is detected, the last known stable lateral acceleration is taken as the result.

<img src="./src/graphics/steering_ramp.png" alt="Steering Ramp" width="500"/>



# Tutorial
## Setup and Build Instructions

This toolchain is now **pure Python** — no ROS 2, Colcon, or C++ build step is required. It runs natively on macOS (Apple Silicon / arm64) and Linux. The only requirement is **Python 3.11 or newer** (it uses the standard-library `tomllib` module for parameter files).

### 1. Create a Python Virtual Environment

Run the following command to create a virtual environment named venv:
```
python3 -m venv venv
```

### 2. Activate the Virtual Environment

Activate the virtual environment with:
```
source venv/bin/activate
```
### 3. Install Dependencies

Once the virtual environment is activated, install the required packages:
```
pip install -r requirements.txt
```
That's it — there is nothing to build. On macOS you do not need `COLCON_IGNORE`, `colcon build`, or `source install/setup.bash`.

> Note: matplotlib uses the non-interactive "Agg" backend by default, so plots are written to PNG files instead of opening windows. This avoids issues with macOS process forking.

## Run the Simulation
1. Check/Modify the parameter files located in the ``./params`` folder (see below).
2. Run the generation script:
```
python3 main_gggv_generation.py
```
Output files are created in the ``output/<timestamp>`` folder (the folder is created if it does not exist):
- ``output/<timestamp>/data/gg_array.npy`` — raw g-g-g-v results array (max lateral acceleration per v/ax/gt combination, plus feasibility/stability flags)
- ``output/<timestamp>/data/g_list.npy``, ``v_list.npy`` — axis value lists for the saved array
- ``output/<timestamp>/plots/gggv.png`` — the generated GGGV plot
- ``output/<timestamp>/params/`` — copy of the parameter files used for the run

Parallelism uses a ``ProcessPoolExecutor`` with the ``spawn`` start method (safe on macOS arm64); the number of workers defaults to the available CPU cores.

### Parameter Files (`params/*.toml`)
- ``veh_default.toml`` — vehicle parameters provided by your car (mass, wheelbase, tire/friction limits, drag, etc.). Point-mass validation model values map directly onto these keys.
- ``sim_default.toml`` — simulation settings (integration time step, ramp rise-rate targets, stability thresholds).
- ``gggv_default.toml`` — the sweep ranges: lists of velocities ``v``, longitudinal accelerations ``ax``, and vertical accelerations ``gt`` to evaluate. Edit these lists to match the operating range of your car; the batch runs over every combination.

## Analyzing the results
The script ``plot_comparison.py`` can be used to compare one or multiple simulation runs with each other. Simply run the script and follow the instructions: a folder-selection dialog lets you pick output directories (the tool loads the ``data/*.npy`` files inside each), and an interactive CustomTkinter/matplotlib UI with sliders lets you select the vertical acceleration (gt) to display for each run so they can be overlaid. The comparisons are based on the diamond shape of the limitations.

```
python3 plot_comparison.py
```

## Debugging Single Simulation Steps
To look into a single simulation run you can switch the program from batch execution to simulate a single steering ramp. Instead of editing the source, pass CLI arguments:

```
python3 main_gggv_generation.py --single --v 40 --ax 0 --gt 9.0
```

- ``--single`` — run one steering-ramp simulation instead of the full batch
- ``--v`` — target velocity [m/s]
- ``--ax`` — target longitudinal acceleration [m/s²]
- ``--gt`` — target vertical acceleration [m/s²]

The result (e.g. the maximum achieved lateral acceleration) is printed to the console and the run's trace/debug data is written under the newest ``output/<timestamp>`` folder.

## Vehicle Dynamics Model
The vehicle dynamics model included in this repository is a simple validation vehicle model, which can be solved analytically. The model is depicted in the figure below. It is implemented in pure Python in ``src/vehicle_model_py.py`` (point-mass model with kinematic steering relation, force saturation to a friction circle shifted down by 2 m/s² to emulate drag, integrated with RK4).

<img src="./src/graphics/validation_model.png" alt="Validation Vehicle Model" width="400"/>

The validation vehicle model point mass model with a kinemtaic steering relation. It is force saturated to form a circle, which is moved downwards by 2 mps2 to simulate drag forces.

## Integrating Your Own Vehicle Model
To use your own vehicle model with this toolchain, you need to provide a model that can be actuated by wheel torques and a steering angle. Furthermore, your model needs to allow external forces to be superimposed onto the center of gravity. The overall simulation process is depicted in the figure below.

<img src="./src/graphics/block_diagram.png" alt="Simulation Process" width="600"/>

1. In the `sim_handler` class in the file 'src/sim_handling.py', instantiate your vehicle model in the `__init__` function of the sim_handler class (replace the ``VehicleModelPy`` instance from ``src/vehicle_model_py.py``).
2. Update the following member functions of the sim_handler class to your model's naming conventions:
    - get_veh_params()
    - set_parameter()
    - set_driver_input()
    - get_simstate()
3. Implement a step function for your model equivalent to ``VehicleModelPy.step(dt)`` returning the updated state (or adapt the call site in `sim_handling.py`).
