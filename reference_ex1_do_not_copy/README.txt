Drone Mapper - Assignment 1

Students:
Name: Shalev Cohen
ID:207610130

Name: Hila Regev Shabath
ID: 215664087

Description:
This project implements a software simulator for an autonomous drone mapper.

The drone maps a 3D building by moving inside the simulated world and scanning its surroundings with a mock lidar sensor.
The drone does not have direct access to the real map. The real map is used only by the simulator, mock lidar sensor, and mock movement driver.

The program loads configuration files, runs the mapping simulation, writes an output map, compares it to the input map, and prints a mapping score.

--------------------------------------------------
Project Structure
--------------------------------------------------

Main files and folders:

src/
   ConfigParser.cpp
   GridMap.cpp
   MapFileWriter.cpp
   MockMovementDriver.cpp
   ScoreCalculator.cpp
   SparseBuildingMap.cpp
   drone.cpp
   GroundTruthMap.cpp
   IMovementDriver.cpp
   main.cpp
   MockLidarSensor.cpp
   MockPositionSensor.cpp
   simulator.cpp
   
include/
    simulator.h
    Drone.h
    ConfigParser.h
    GroundTruthMap.h
    SparseBuildingMap.h
    MockPositionSensor.h
    MockLidarSensor.h
    MockMovementDriver.h
    DroneConfig.h
    MissionConfig.h
    DroneState.h
    Types.h
    Units.h
    IPositionSensor.h
    ILidarSensor.h
    IMovementDriver.h
    IBuildingMap.h

tools/
    visualize_drone_path.py

Input files:
    drone_config.txt
    mission_config.txt
    map_input.txt

Output files:
    map_output.txt
    input_errors.txt          Created only if recoverable input errors exist.
    drone_log.csv
    drone_path_visualization.html

--------------------------------------------------
Input Files
--------------------------------------------------

The program expects the following files in the input/output directory:

1. drone_config.txt
   Contains the drone capabilities, such as:
   - maximum advance distance
   - maximum elevate distance
   - maximum rotation angle
   - lidar range
   - lidar resolution / field of view
   - minimum pass width
   - minimum pass height

2. mission_config.txt
   Contains mission-specific information, such as:
   - start position
   - start angle
   - mapping boundaries
   - minimum and maximum height
   - required map resolution

3. map_input.txt
   Contains the real simulated building map.
   This map is hidden from the drone and is used only by the simulator and mock sensors.

--------------------------------------------------
Output Files
--------------------------------------------------

1. map_output.txt
   The map discovered by the drone.

2. input_errors.txt
   Created only if the input files contain recoverable errors.
   The program uses default values when possible and writes a description of the recovered errors.

3. drone_log.csv
   A log of the drone's path and actions during the simulation.

4. drone_path_visualization.html
   Optional visual output created by the Python visualization tool.

--------------------------------------------------
Input and Output File Formats
--------------------------------------------------

Input file format:

1. drone_config.txt

The file contains one line with 10 numeric values:

maxAdvance maxElevate maxRotate lidarMinRange lidarMaxRange lidarBeamSpacing lidarFovCircleCount minPassageWidth minPassageLength minPassageHeight

Example:

100 50 90 0 10 1 1 2 2 2

Meaning:
- maxAdvance: maximum distance the drone can advance in one movement command.
- maxElevate: maximum distance the drone can move up or down in one elevation command.
- maxRotate: maximum angle, in degrees, the drone can rotate in one rotation command.
- lidarMinRange: minimum reliable lidar range.
- lidarMaxRange: maximum lidar range.
- lidarBeamSpacing: spacing between lidar beams.
- lidarFovCircleCount: number of lidar field-of-view circles.
- minPassageWidth: minimum passage width the drone can pass through.
- minPassageLength: minimum passage length the drone can pass through.
- minPassageHeight: minimum passage height the drone can pass through.

2. mission_config.txt

The file contains one line with 11 numeric values:

startX startY startHeight startAngle minX maxX minY maxY minHeight maxHeight resolution

Example:

2 2 1 0 0 4 0 4 0 2 1

Meaning:
- startX, startY, startHeight: initial drone center position.
- startAngle: initial horizontal angle of the drone, in degrees.
- minX, maxX: mission boundary in the X direction.
- minY, maxY: mission boundary in the Y direction.
- minHeight, maxHeight: mission boundary in the height direction.
- resolution: grid resolution of the map.

3. map_input.txt

The file starts with three integers:

width height depth

Then it contains depth layers.  
Each layer contains height rows.  
Each row contains width cell values.

Cell values:
- 0 means free space.
- 1 means occupied space / obstacle.

Example for a 3x2x2 map:

3 2 2
0 0 0
0 1 0

0 0 0
1 0 0

The first layer represents height level 0, the second layer represents height level 1, and so on.

Output file format:

1. map_output.txt

The output map has the same format as map_input.txt:

width height depth

followed by depth layers, each containing height rows and width values.

Cell values:
- -1 means unknown / unmapped.
- 0 means free.
- 1 means occupied.
- 2 out of bounds .

The drone writes this file according to the map it discovered during the simulation.

2. input_errors.txt

This file is created only if recoverable input errors were found.
Each line describes an input problem and the default or recovery action used by the program.

3. drone_log.csv

This file contains a CSV log of the drone simulation.

Format:

step,event,x,y,height,angle

Example:

0,start,2 cm,2 cm,1 cm,0°
1,get_location,2 cm,2 cm,1 cm,0°
2,scan,2 cm,2 cm,1 cm,0°
3,advance_success,3 cm,2 cm,1 cm,0°

Meaning:
- step: simulation step number.
- event: action or result at that step.
- x, y, height: drone position.
- angle: drone horizontal angle.

4. drone_path_visualization.html

This is an optional HTML visualization created from drone_log.csv by the Python visualization tool.

--------------------------------------------------
Build Instructions
--------------------------------------------------

From the project root directory, run:

mkdir -p build
cd build
cmake ..
make

This should create the executable:

drone_mapper

--------------------------------------------------
Run Instructions
--------------------------------------------------

The program can be run with an optional path argument:

./drone_mapper [input_output_files_path]

If no path is given, the current directory is used.

--------------------------------------------------
Expected Input File Names
--------------------------------------------------

The program searches the given input/output path for:

drone_config.txt
mission_config.txt
map_input.txt

The program creates or overwrites:

map_output.txt

--------------------------------------------------
Visualization
--------------------------------------------------

If the drone path log exists, the visualizer can be run from the project root:

python3 ./tools/visualize_drone_path.py

Or, if the files are in another directory:

python3 ./tools/visualize_drone_path.py [input_output_files_path]

The visualizer creates:

drone_path_visualization.html

To view it, open the HTML file 
