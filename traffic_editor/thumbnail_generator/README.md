# Traffic Editor Model Thumbnail Generator

This is the pipeline for generating thumbnails to be used in `traffic_editor`.

# Prerequisites

* ROS Melodic
* Gazebo-9

Install all the other required packages, **and be sure to build all packages in the `traffic_editor` package**

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
* make sure to include all your necessary model, plugin, resource paths in `scripts/start_gazebo_screen.sh`, otherwise any custom models will not be found or spawned.

Create a green screen world in `gazebo`, and start spawning models listed in your `model_list.yaml`. The scripts will take care of spawning the models one-at-a-time inside the green screen world and record an approximate orthogonal view of them.

```bash
# launch the world in a terminal
./scripts/start_gazebo_screen.sh green

# start the spawning and photo-taking script in a second terminal
./scripts/top_view_generator.py test/model_list.yaml images/green
```

The orthogonal views of the models with a green screen will have been saved in the folder `images/green/`. Next, do the same with a white screen, 

```bash
# launch the world in a terminal
./scripts/start_gazebo_screen.sh white

# in a second terminal
./scripts/top_view_generator.py test/model_list.yaml images/white
```

Similarly, the folder `images/white/` will have been populated when the script finishes.

The contours of the models are then extracted from the green-screened images using OpenCV to generate a transparency mask, used for cropping the white-background images, so that it stays centered and is as small as possible. This can be done by calling,

```bash
./scripts/crop.py test/model_list.yaml -o images/cropped -g images/green -w images/white
```

The generated thumbnails in `images/cropped` can then be added to `traffic_editor/thumbnails/images/`, while the model names need to be updated to `traffic_editor/thumbnails/model_list.yaml`.

# Utilities

A new model list `.yaml` file can be generated using the utility script, where an optional blacklisted model names can be added, to avoid creating moving models or agents,

```bash
./scripts/generate_model_list.py output_model_list.yaml -d MODEL_DIR -b test/model_blacklist.yaml
```

In the event that merging multiple model lists is required, a different utility script can be used,

```bash
./scripts/merge_model_lists.py test/merged_model_list.yaml -s test/extra_model_list.yaml
```

# Credits

Created by [codebot](https://github.com/codebot).
