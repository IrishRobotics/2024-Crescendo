# :sparkles::notes: FRC 2606 :notes::sparkles:

Team 2606's 2404 FRC robot code for *Crescendo*. *Crescendo*'s code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
- Clone this repo
- Run `./gradlew` to download gradle and needed FRC libraries
- Run `./gradlew tasks` to see available build options
- Enjoy!

### Building/Deploying to the Robot
- Run `./gradlew build` to build the code. Use the `--info` flag for more details
- Run `./gradlew deploy -PteamNumber=2509` to deploy to the robot in Terminal (Mac) or Powershell (Windows)

### Wiring Compontents Diagram

Subsystem  | Controller       | Name           | ID     | PDP  |
---------- | ---------------- | -------------- | ------ | ---- |
           |                  |                |        | ---- |


## Code Highlights

- Building with Gradle

	Instead of working with Ant, we used GradleRIO, which is a powerful Gradle plugin that allows us to build and deploy our code for FRC. It automatically fetches WPILib, CTRE Toolsuite, and other libraries, and is easier to use across different IDEs. 


## Package Functions
- frc.robot

	Contains the robot's central functions and holds a file with all numerical constants used throughout the code. For example, the `Robot` class controls all routines depending on the robot state.

- frc.robot.subsystems
	
	Subsystems are consolidated into one central class per subsystem, all of which extend the Subsystem abstract class. Each subsystem uses state machines for control.
	Each subsystem is also a singleton, meaning that there is only one instance of each. To modify a subsystem, one would get the instance of the subsystem and change its state. The `Subsystem` class will work on setting the desired state.

- frc.robot.commands

	Commands define the operation of the robot incorporating the capabilities defined in the subsystems. Commands are subclasses of `Command` or `CommandGroup`. Commands run when scheduled or in response to buttons being pressed or virtual buttons from the `SmartDashboard`.



## Variable Naming Conventions
- c*** (i.e. `cAutonomous`): Command instance variables
- k*** (i.e. `kDriveWheelTrackWidthInches`): Final constants, especially those found in the Constants.java file
- m*** (i.e. `mIsHighGear`): Private instance variables
- s*** (i.e. `sDrivetrain`): Subsystems variables, especially those found in Robot.java file

## PID Tuning Method
1. Start by setting `I` and `D` to 0.
2. Increase `P` until the system starts oscillating for a period of `Tu`. You want the oscillation to be large enough that you can time it. This maximum `P` will be referred to as `Ku`.
3. Use the chart below to calculate different `P`, `I`, and `D` values.

Control Types | P | I | D |
------------- | - | - | - |
P | .5*`Ku` | 0 | 0 |
PI | .45*`Ku` | .54*`Ku`/`Tu` | 0 |
PID | .6*`Ku` | 1.2*`Ku`/`Tu` |	3*`Ku`*`Tu`/40 |

## Programmers

### Students

### Mentors
[Nate](https://github.com/naterbots)<br/>