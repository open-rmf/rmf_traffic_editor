![](https://github.com/open-rmf/rmf_traffic_editor/workflows/build/badge.svg)

# Traffic Editor
A graphical editor for robot traffic flows. The intent is to make it easy
to annotate building floorplans with the desired robot traffic lanes and
generate simulation models to test and evaluate different traffic schemes.

## Quality Declaration

This package claims to be in the **Quality Level 3** category.
See the [Quality Declaration](./QUALITY_DECLARATION.md) for more details.

## System Requirements

This program is developed and tested on
[Ubuntu 18.04 LTS](http://releases.ubuntu.com/18.04/), using
[Qt 5](https://doc.qt.io/qt-5/qt5-intro.html) and
[`yaml-cpp`](https://github.com/jbeder/yaml-cpp).

## Compiling Instructions
Traffic Editor is now structured as a Colcon package. After installing
ROS 2 Eloquent, the following command sequence will create a colcon
workspace in `~/colcon_workspace` and build `traffic-editor` there:

```bash
sudo apt update
sudo apt install libyaml-cpp-dev qt5-default \
  libopencv-dev libopencv-videoio-dev
mkdir -p ~/colcon_workspace/src
cd ~/colcon_workspace/src
git clone https://github.com/open-rmf/rmf_traffic_editor
cd ~/colcon_workspace
source /opt/ros/eloquent/setup.bash
colcon build --packages-select rmf_traffic_editor
```

The companion `traffic_editor_assets` package contains a nifty bunch of useful assets to use with `rmf_traffic_editor`.
It is included in the above checkout.

Then you should be able to run `traffic-editor` by sourcing the install
space of that workspace, in a new "clean" terminal:
```bash
source ~/colcon_workspace/install/setup.bash
traffic-editor
```

# Quick Start

If it's the first time you are running it, starting the editor with
`traffic-editor` should bring up a blank window.

First, you'll need to make sure that `traffic-editor` knows where the
model thumbnails are found. If you installed `traffic_editor_assets`,
`rmf_traffic_editor` should find it automatically. The thumbnails are
top-view renderings of various art assets that can be added to the
environments, such as chairs.

Click `Edit->Preferences...` and see if the path provided in the "Thumbnail Path" box looks reasonable. 

If necessary, the "Find..." button can be used to browse the filesystem to point to any desired thumbnail directory.

(If you installed the recommended `traffic_editor_assets` package, you will find its thumbnail directory in its install space at `<workspace_dir>/install/traffic_editor_assets/share/assets/thumbnails`.)

### Creating a new Project and an empty Building Map

Click `Project->New...` and save your new project as `test.project.yaml`

Click `Edit->Project Properties...` and enter "test" as the project name and `test.building.yaml` as the building path. Then click OK.

Click `Edit->Building Properties...` and enter "test" as the building name.
Click OK.

### Creating a level and adding some stuff

Click the "Add..." button in the "levels" tab on the far right side of the main editor window. This will pop up a dialog where you can create a new level. Enter `L1` for the name and click OK. This will create a 10 meter square level.

You can zoom in and out using the mouse wheel on the rendering on the left side of the main window. You can pan around by dragging the mouse around with the mouse wheel (or middle button) depressed.

Now, you should be able to click the green dot toolbar icon, which is the "Add Vertex" tool (or press `V`) and click a few vertices in the white area. Press the `[Escape]` key to return to the "Select" tool.

Now, you should be able to click the `add wall` tool (or press `W`) and drag from one vertex to another vertex to add wall segments.

To delete wall segments or vertices, first press `[Escape]` to enter Select mode. Then, click on a wall segment or vertex, and press `[Delete]`.

### Save your work

Click `Project->Save` or press `Ctrl+S` to save the project and building map.

### Adding real-world measurements to set the scale

To set the scale of the drawing, click the `add measurement` tool (or press `M`) and drag from one vertex to another to add a real-world measurement line, which should show up as a pink line. Then click the `select` tool (or press `Esc`) and click on the line with the left button. This should populate the property-editor in the lower-right pane of the editor window. You can then specify the real-world length of the measurement line in meters. If you set more than one measurement line on a drawing, the editor will compute an average value of pixels-per-meter from all supplied measurements.

Currently you need to re-load the document (closing the editor and re-opening) to re-compute the scale. This is not ideal, but is hopefully not a frequently-used feature. Typically the scale of a map is only set one time.

### Adding lifts

Click the "Add..." button in the "lifts" tab on the far right side of the main editor window. This will pop up a dialog where you can create a new lift. You can specify the name, position, size, and reference floor in the dialog.

*Note: Do include the keywork "lift" in the lift name as for now this is how slotcars recognize lift models.*

You can add lift doors by lick the "Add..." button below the box showing the lift. Set Door type to "Double sliding" (The only supported type for now!), and align the doors to the edge of the lift (represented by the green box). After that, select which door you want to use on each floor by simply checking the boxes on the left.

Lift waypoints at the center of the lift on each level can also be generated using the "Add lift waypoints" button in the dialog. Note that waypoints will only be generated on levels that the lift is serving (has a door opening on that level).

### Generating Custom Thumbnails

Model thumbnails are used in `rmf_traffic_editor`. To generate a thumbnail, a simple working example is shown here to generate a `SUV`:
```bash
# Run as gz plugin, set --a for help options printout
gzserver -s libthumbnail_generator.so empty.world --input ~/.gazebo/models/SUV/model.sdf --output .
```
After execution, you will notice a newly created `SUV.png` in your current working directory. This can be further placed into `traffic_editor_assets/assets/thumbnails`.

To generate multiple model thumbnails listed in `model_list.yaml`, run this:
```bash
export GAZEBO_MODEL_PATH=/PATH/TO/MODELS; ./scripts/generate_thumbnails.py /PATH/TO/MODELS test/model_list.yaml ~/output
```

User can also change the script default configs:  `img_size`, `cam_height` and `fhov`, which will alter the `meters_per_pixel` value.

Similarly, the generated thumbnails in `~/output` can then be added to `traffic_editor_assets/assets/thumbnails`, while also append `model_list.yaml`.

### Utilities

A new model list `.yaml` file can be generated using the utility script, where an optional blacklisted model names can be added, to avoid creating moving models or agents,

```bash
# e.g. MODEL_DIR = '~/.gazebo/models'
./scripts/generate_model_list.py output_model_list.yaml -d MODEL_DIR -b test/model_blacklist.yaml
```

In the event that merging multiple model lists is required, a different utility script can be used,

```bash
./scripts/merge_model_lists.py output_model_list.yaml -s test/model_list.yaml
```

To sort the model list `.yaml` file,

```bash
./scripts/sort_model_list.py model_list.yaml
```
