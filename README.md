# POLARIS GEM Simulator Notes

Repo:
https://github.com/harishkumarbalaji/POLARIS_GEM_Simulator

This README reflects the workflow that was actually validated on this machine.

## Workspace Setup

```bash
mkdir -p ~/gem_simulation_ws/src
cd ~/gem_simulation_ws/src
git clone https://github.com/harishkumarbalaji/POLARIS_GEM_Simulator.git
cd ~/gem_simulation_ws/src/POLARIS_GEM_Simulator
```

Check NVIDIA Docker:

```bash
sudo docker run --rm --gpus all ubuntu nvidia-smi
```

Build the Docker image:

```bash
bash setup/build_docker_image.sh
```

## Start Container

From the simulator repo:

```bash
cd ~/gem_simulation_ws/src/POLARIS_GEM_Simulator
bash run_docker_container.sh
```

Inside the container:

```bash
cd ~/host/gem_simulation_ws
catkin_make
source devel/setup.bash
```

Stop the container:

```bash
cd ~/gem_simulation_ws/src/POLARIS_GEM_Simulator
bash stop_docker_container.sh
```

## Important Path Rule

Inside the container, the host home directory is mounted at:

```bash
/home/$USER/host
```

That means:

- Host path: `~/teacher_paths/teacher_path.csv`
- Container path: `/home/$USER/host/teacher_paths/teacher_path.csv`

Same for datasets:

- Host path: `~/pilotnet_data/...`
- Container path: `/home/$USER/host/pilotnet_data/...`

If you save to `~/teacher_paths` or `~/pilotnet_data` inside the container, the files stay in the container filesystem and will be lost when the container is removed.

## Launch Scene

For teacher-path recording and dataset collection, use `custom_scene:=false`.
`custom_scene:=true` was unstable in this setup and could crash `gzserver`.

```bash
source devel/setup.bash
roslaunch gem_launch gem_init.launch world_name:="highbay_track.world" x:=12.5 y:=-21 yaw:=3.1416 custom_scene:=false
```

Reset vehicle pose:

```bash
cd ~/host/gem_simulation_ws/src/POLARIS_GEM_Simulator
python3 utils/set_pos.py --x 12.5 --y -21 --yaw 3.1416
```

## Record Teacher Path

Open one terminal in the container:

```bash
source devel/setup.bash
rosrun gem_gazebo record_teacher_path.py \
  _output_csv:=/home/$USER/host/teacher_paths/teacher_path.csv
```

Open another terminal in the container for manual driving:

```bash
source devel/setup.bash
cd ~/host/gem_simulation_ws/src/POLARIS_GEM_Simulator
python3 -m pip install --user pynput
python3 utils/generate_waypoints.py _max_speed:=0.5
```

Use `w a s d` to drive. After one clean lap, stop the recorder with `Ctrl+C`.

The path will be saved on the host at:

```bash
~/teacher_paths/teacher_path.csv
```

## Teacher Follow

Run the teacher controller inside the container:

```bash
source devel/setup.bash
rosrun gem_gazebo teacher_path_follow.py \
  _path_csv:=/home/$USER/host/teacher_paths/teacher_path.csv \
  _speed:=0.45 \
  _lookahead:=2.0 \
  _max_steering:=0.55
```

## Collect PilotNet Data

Run data collection inside the container while the teacher is driving:

```bash
source devel/setup.bash
rosrun gem_gazebo collect_pilotnet_data.py \
  _output_root:=/home/$USER/host/pilotnet_data
```

This writes to the host:

```bash
~/pilotnet_data/
```

Stop collection with `Ctrl+C`.

Check the data on the host:

```bash
find ~/pilotnet_data -name metadata.csv
```

## Train PilotNet

Training is run on the host, not in the ROS container.

From the repo training directory:

```bash
cd ~/gem_simulation_ws/src/POLARIS_GEM_Simulator/pilotnet
```

Create a local `uv` environment:

```bash
uv venv --python 3.10
source .venv/bin/activate
uv pip install torch torchvision torchaudio --torch-backend=auto
uv pip install pillow numpy
```

Check PyTorch:

```bash
python -c "import torch; print(torch.__version__); print(torch.version.cuda); print(torch.cuda.is_available())"
```

Train:

```bash
python train_pilotnet.py \
  --data-root ~/pilotnet_data \
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

Including:

- `best_model.pt`
- `latest_model.pt`
- `metrics.json`
- `train_args.json`

## Run PilotNet Inference

Run this inside the container after `catkin_make` and `source devel/setup.bash`.

The checkpoint path below uses the container view of the host repository.

Pre-requsite
TODO: MOve to Dockerfile
```bash
# Compatible with Python 3.8
python3 -m pip install --upgrade pip
python3 -m pip install --user torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cu124
python3 -m pip install pillow numpy
# Verify
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"


```

```bash
cd ~/host/gem_simulation_ws/src/POLARIS_GEM_Simulator
python3 utils/set_pos.py --x -5.5 --y -21 --yaw 3.1416

source devel/setup.bash
rosrun gem_gazebo pilotnet_inference.py \
  _checkpoint:=/home/$USER/host/gem_simulation_ws/src/POLARIS_GEM_Simulator/pilotnet/runs/run_001/best_model.pt \
  _speed:=0.35 \
  _max_steering:=0.55 \
  _steering_scale:=1.0 \
  _steering_smoothing:=0.25
```

Notes:

- This node uses the same crop and resize settings stored in the training checkpoint.
- It publishes directly to `/ackermann_cmd`.
- If you want more aggressive steering, increase `_steering_scale`.
- If the steering is too noisy, increase `_steering_smoothing`.

## Files Added For This Pipeline

Training directory:

- `pilotnet/pilotnet_model.py`
- `pilotnet/train_pilotnet.py`
- `pilotnet/README.md`

Simulator repo:

- `gem_simulator/gem_gazebo/scripts/record_teacher_path.py`
- `gem_simulator/gem_gazebo/scripts/teacher_path_follow.py`
- `gem_simulator/gem_gazebo/scripts/collect_pilotnet_data.py`
- `gem_simulator/gem_gazebo/scripts/pilotnet_inference.py`
- `gem_simulator/gem_gazebo/CMakeLists.txt`
- `run_docker_container.sh`

## Common Pitfalls

- Run GUI-related commands from the RustDesk desktop terminal, not plain SSH.
- Inside the container, save persistent outputs under `/home/$USER/host/...`.
- If `teacher_path_follow.py` says the path file is missing, you are probably using a host path instead of the container path.
