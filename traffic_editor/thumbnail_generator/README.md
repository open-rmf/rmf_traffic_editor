# Traffic Editor Model Thumbnail Generator

This is the pipeline for generating thumbnails to be used in `traffic_editor`.

# Prerequisites

* ROS Melodic
* Gazebo-9

Install all the other required packages,

```bash
sudo apt install \
  ros-melodic-gazebo-ros \
  ros-melodic-gazebo-ros-pkgs \
  ros-melodic-gazebo-msgs \
  ros-melodic-geometry-msgs \
  ros-melodic-sensor-msgs \
  ros-melodic-cv-bridge
```

Clone the repository and navigate to the repository,

```bash
git clone https://github.com/osrf/traffic_editor
cd traffic_editor/traffic_editor/thumbnail_generator
```

# Instructions

**Notes before running any scripts,**
* take note to clear up or backup any pre-existing thumbnails and images created inside the default folders, `images/green`, `images/white`, `images/cropped`, if they are intended to be used.
* make sure to include all your necessary model, plugin, resource paths in `scripts/start_gazebo_screen.sh`, otherwise any custom models will not be found and spawneds

Create a green screen world in `gazebo`, and start spawning models listed in your `model_list.yaml`. The scripts will take care of spawning the models one-at-a-time inside the green screen world and record an approximate orthogonal view of them.

```bash
# launch the world in a terminal
./scripts/start_gazebo_screen.sh green

# start the spawning and photo-taking script in a second terminal
./scripts/top_view_generator.py -m test/model_list.yaml -o images/green
```

The orthogonal views of the models with a green screen will have been saved in the folder `images/green/`. Next, do the same with a white screen, 

```bash
# launch the world in a terminal
./scripts/start_gazebo_screen.sh white

# in a second terminal
./scripts/top_view_generator.py -m test/model_list.yaml -o images/white
```

Similarly, the folder `images/white/` will have been populated when the script finishes.

The contours of the models are then extracted from the green-screened images using OpenCV to generate a transparency mask, used for cropping the white-background images, so that it stays centered and is as small as possible. This can be done by calling,

```bash
./scripts/crop.py -m test/model_list.yaml -g images/green -w images/white -o images/cropped
```

The generated thumbnails in `images/cropped` can then be added to `traffic_editor/thumbnails/images/`, while the model names need to be updated to `traffic_editor/thumbnails/model_list.yaml`.

# Credits

Created by [codebot](https://github.com/codebot).
