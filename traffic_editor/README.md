![](https://github.com/osrf/traffic_editor/workflows/build/badge.svg)

# Traffic Editor
A graphical editor for robot traffic flows. The intent is to make it easy
to annotate building floorplans with the desired robot traffic lanes and
generate simulation models to test and evaluate different traffic schemes.

## System Requirements

This program is developed and tested on
[Ubuntu 18.04 LTS](http://releases.ubuntu.com/18.04/), using
[Qt 5](https://doc.qt.io/qt-5/qt5-intro.html) and
[`yaml-cpp`](https://github.com/jbeder/yaml-cpp).

## Compiling Instructions
Configuring and compiling can be sequenced using `colcon` or using
vanilla CMake:
```
sudo apt update
sudo apt install git cmake libyaml-cpp-dev qt5-default
git clone https://github.com/osrf/traffic_editor
cd traffic_editor/traffic_editor
mkdir build
cd build
cmake ..
make
```

# Quick Start

If it's the first time you are running it, starting the editor with
`./traffic-editor` should bring up a blank window.

First, you'll need to make sure that `traffic-editor` knows where the model
thumbnails are found. Depending on how `traffic-editor` is built, it may not
find it automatically. The thumbnails are top-view renderings of various art
assets that can be added to the environments, such as chairs. Click
"Edit->Preferences..." and see if the path provided in the "Thumbnail Path"
box looks reasonable. If necessary, the "Find..." button can be used to browse
the filesystem to point to the "traffic_editor/thumbnails" directory in the
`traffic_editor` repository that was cloned earlier.

### Creating a new Project and an empty Building Map

Click `Project->New...` and save your new project as `test.project.yaml`

Click `Edit->Project Properties...` and enter "test" as the project name
and `test.building.yaml` as the building path. Then click OK.

Click `Edit->Building Properties...` and enter "test" as the building name.
Click OK.

### Creating a level and adding some stuff

Click the "Add..." button in the "levels" tab on the far right side of the
main editor window. This will pop up a dialog where you can create a new
level. Enter `L1` for the name and click OK. This will create a 10 meter square
level.

You can zoom in and out using the mouse wheel on the rendering on the left
side of the main window. You can pan around by dragging the mouse around with
the mouse wheel (or middle button) depressed.

Now, you should be able to click the green dot toolbar icon, which is the "Add
Vertex" tool (or press `V`) and click a few vertices in the white area. Press
the `[Escape]` key to return to the "Select" tool.

Now, you should be able to click the `add wall` tool (or press `W`) and
drag from one vertex to another vertex to add wall segments.

To delete wall segments or vertices, first press `[Escape]` to enter Select
mode. Then, click on a wall segment or vertex, and press `[Delete]`.

### Save your work

Click `Project->Save` or press `Ctrl+S` to save the project and building map.

### Adding real-world measurements to set the scale

To set the scale of the drawing, click the `add measurement` tool (or
press `M`) and drag from one vertex to another to add a real-world measurement
line, which should show up as a pink line. Then click the `select` tool (or
press `Esc`) and click on the line with the left button. This should populate
the property-editor in the lower-right pane of the editor window. You can then
specify the real-world length of the measurement line in meters. If you set
more than one measurement line on a drawing, the editor will compute an average
value of pixels-per-meter from all supplied measurements.

Currently you need to re-load the document (closing the editor and re-opening)
to re-compute the scale. This is not ideal, but is hopefully not a
frequently-used feature. Typically the scale of a map is only set one time.
