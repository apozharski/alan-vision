# ALAN-vision
This is the repository that contains the code for the vision processing subsystem for ALAN. 

## Compiling
Compilation currently requires the following:
- A system installation of OpenCV
- A CMake installation of version 3.0 or later.

And compiles against **C++14**. In order to compile do the following:
```
mkdir build
cd build
cmake ..
make
```
This will make the main program that runs the vision system. In future other dependencies will be added.
