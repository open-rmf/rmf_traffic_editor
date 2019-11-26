![](https://github.com/osrf/traffic_editor/workflows/ci/badge.svg)

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
ln -s ../thumbnails .
cmake ..
make
```

# Quick Start

Starting the editor with `./traffic-editor` should bring up a blank window.

Click `File->New...` and save your new map as something like `test`

Click `Level->Add...` to create a new level, calling it something like `L1`

Then you should be able to click the `add vertex` tool (or press `V`) and
click a few vertices in the white area. You can use the mouse wheel to zoom,
and drag with the middle-button (wheel) to pan.

Then you should be able to click the `add lane` tool (or press `L`) and
drag from one vertex to another vertex to add a traffic lane.

Similarly, you should be able to click the `add wall` tool (or press `W`) and
drag from one vertex to another to add a wall.

To set the scale of the drawing, click the `add measurement` tool (or
press `M`) and drag from one vertex to another to add a real-world measurement
line, which should show up as a pink line. Then click the `select` tool (or
press `Esc`) and click on the line with the left button. This should populate
the property-editor in the rightmost pane of the editor window. You can then
specify the real-world length of the measurement line in meters. If you set
more than one measurement line on a drawing, the editor will compute an average
value of pixels-per-meter from all supplied measurements.

Currently you need to re-load the document (closing the editor and re-opening)
to re-compute the scale. (non-ideal... but hopefully not frequently required)
