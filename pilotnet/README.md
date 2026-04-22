# PilotNet Workflow

This directory contains the host-side training code for the camera-only PilotNet pipeline used with the ROS/Gazebo simulator in this repository.

The ROS-side nodes live in:

- `gem_simulator/gem_gazebo/scripts/record_teacher_path.py`
- `gem_simulator/gem_gazebo/scripts/teacher_path_follow.py`
- `gem_simulator/gem_gazebo/scripts/collect_pilotnet_data.py`
- `gem_simulator/gem_gazebo/scripts/pilotnet_inference.py`

## Path Rule

Inside the Docker container, the host home is mounted at:

```bash
/home/$USER/host
```

That means:

- Host: `/home/yuwei/teacher_paths/teacher_path.csv`
- Container: `/home/yuwei/host/teacher_paths/teacher_path.csv`

- Host: `/home/yuwei/pilotnet_data/...`
- Container: `/home/yuwei/host/pilotnet_data/...`

If you write to `~/teacher_paths` or `~/pilotnet_data` inside the container, the data stays in the container filesystem and may be lost when the container is removed.

## 1. Launch Scene

For teacher recording and data collection, use:

```bash
source devel/setup.bash
roslaunch gem_launch gem_init.launch world_name:="highbay_track.world" x:=12.5 y:=-21 yaw:=3.1416 custom_scene:=false
```

`custom_scene:=true` was unstable during validation and could crash `gzserver`.

## 2. Record Teacher Path

Inside the container:

```bash
source devel/setup.bash
rosrun gem_gazebo record_teacher_path.py \
  _output_csv:=/home/$USER/host/teacher_paths/teacher_path.csv
```

Open another terminal in the same container for manual driving:

```bash
source devel/setup.bash
cd ~/host/gem_simulation_ws/src/POLARIS_GEM_Simulator
python3 -m pip install --user pynput
python3 utils/generate_waypoints.py _max_speed:=0.5
```

Drive one clean lap with `w a s d`. Stop the recorder with `Ctrl+C`.

## 3. Run Teacher Controller

Inside the container:

```bash
source devel/setup.bash
rosrun gem_gazebo teacher_path_follow.py \
  _path_csv:=/home/$USER/host/teacher_paths/teacher_path.csv \
  _speed:=0.45 \
  _lookahead:=2.0 \
  _max_steering:=0.55
```

## 4. Collect Training Data

Inside the container:

```bash
source devel/setup.bash
rosrun gem_gazebo collect_pilotnet_data.py \
  _output_root:=/home/$USER/host/pilotnet_data
```

The dataset will be saved on the host under:

```bash
/home/yuwei/pilotnet_data/
```

## 5. Train on Host

From this directory:

```bash
cd ~/gem_simulation_ws/src/POLARIS_GEM_Simulator/pilotnet
```

Create an environment with `uv`:

```bash
uv venv --python 3.10
source .venv/bin/activate
uv pip install torch torchvision torchaudio --torch-backend=auto
uv pip install pillow numpy
```

Train:

```bash
python train_pilotnet.py \
  --data-root /home/yuwei/pilotnet_data \
  --output-dir runs/run_001 \
  --epochs 20 \
  --batch-size 256 \
  --learning-rate 1e-3 \
  --num-workers 4
```

Outputs are written to:

```bash
runs/run_001/
```

## 6. Run PilotNet Inference

Inside the container, after `catkin_make` and `source devel/setup.bash`:

```bash
source devel/setup.bash
rosrun gem_gazebo pilotnet_inference.py \
  _checkpoint:=/home/$USER/host/gem_simulation_ws/src/POLARIS_GEM_Simulator/pilotnet/runs/run_001/best_model.pt \
  _speed:=0.35 \
  _max_steering:=0.55 \
  _steering_scale:=1.0 \
  _steering_smoothing:=0.25
```

## Notes

- The current model uses only the front camera topic `/oak/rgb/image_raw`.
- Training predicts only `steering_angle`. Speed remains fixed at inference time.
- If you only have one recording session, validation falls back to a single-session frame split and will be optimistic.
