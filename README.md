# traffic-editor
A graphical editor for robot traffic flows. The intent is to make it easy
to annotate building floorplans with the desired robot traffic lanes and
generate simulation models to test and evaluate different traffic schemes.

## compiling instructions
```
sudo apt update
sudo apt install git cmake libyaml-cpp-dev qt5-default
git clone https://github.com/osrf/traffic-editor
cd traffic-editor
mkdir build
ln -s ../thumbnails .
cd build
cmake ..
make
```
