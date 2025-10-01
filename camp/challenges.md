[[_TOC_]]

# FRC Code / Electrical Bootcamp

Start here: [Team 4682 Beginner Coder Resources](https://tinyurl.com/4682StartCoding)

CyBears coders new to FRC will ideally go through training to learn how FRC electronics and code work together.  The general idea behind a boot camp is to accelerate learning.  To attempt to take advantage of a bit of rapid learning, CyBears will therefore attempt to have each coding team member work through a key 'example project'.

The idea behind an example project will have each individual project interact with some form of input and have that signal control a form of robot output.  Successful completion of the project includes both checking in of the full project into the FirstAge2026\camp\\{team member} repo directory as well as demonstrating the functionality checked in works on the project test bed.

# Input and Output Pairings / Assignments

The table below highlights various challanges that coder members will be assigned as their bootcamp example project.

## First Year Coders

|Input Hardware | Output Hardware | Expected Outcome | Target Owner | Owner |
|--|--|--|--|--|
| XBox Controller Left and Right bumper | Talon Motor | Left bumper press drives the motor backwards or when not pressed use right bumper press to drive motor forwards | 1st year | ?? |
| XBox Controller Left Stick Y direction | Neo motor  | Use of the input of the Left stick y double/floating point value to control the speed of the motor | 1st year | ?? |
| XBox Controller Right Stick X direction | Talon Motor | Use of the input of the Right stick y double/ floating point value to control the speed of the motor | 1st year | ?? |
| XBox Controller two buttons (X and B) | bag motor on/off in forward and reverse directions | Use one button to drive the motor in forward direction and the other button press to run the motor in the other direction | 1st year | ?? |
| XBox Controller D-Pad up/down | Pneumatic double soleniod enable/disable | The intent here is to trigger a pneumatic solenoid using the xbox controller D-pad input as boolean values. | 1st year | ??  |

## Second/Third Year Coders (or others already completed AP Comp Science)
|Input Hardware | Output Hardware | Expected Outcome | Target Owner | Owner |
|--|--|--|--|--|
| Mag sensor | bag motor on/off in a single direction | Every time a beam break sensor is broken turn motor on, otherwise turn off motor | 2nd year | ??  |
| Tof/Distance sensor | Small neo | As something comes into view of the sensor closer and closer motor speed increases.  When nothing is in view the motor is off | 2nd year | ??  |
| Pigeon various movements | LED colors| Every time an axis movement on the pigeon occurs one of 6 colors is displayed on the LEDs. | 2nd year | ??  |
| XBox Controller joystick. If left trigger not pressed, full speed. If left trigger pressed, 1/4 speed.  | Talon motor | Use the input of BOTH the left and right triggers as double/floating point value to control the speed of the motor (where left is negative and right is positive).  | 2nd year | ??  |
| Sendable chooser and smart dashboard button | Neo Motor | Implement sendable chooser w/ motor slow/fast/backwards. Button on smart dashboard engages the selected mode. | 2nd year | ??  |
| XBox controller button, Limelight and April Tag | Chassis + 4-wheel Swerve | Add feature to do April Tag following.  Robot should be able to center itself on an April tag that is within the field of vision. | 2nd year | ?? |


# Expected Activity Breakdown

The sections below generally describe the steps that should be followed during bootcamp.

##  Sign Up As the Owner for an Example Project

1. Create a feature branch from the [FirstAge2026 Repo](https://github.com/Team4682CyBears/FirstAge2026)
2. Make an edit to this file [challenges.md](https://github.com/Team4682CyBears/FirstAge2026/blob/main/camp/challenges.md) and add your name as the 'owner' in one of the unassigned '??' above 
3. Create a pull request with the change
4. Have a mentor approve your pull request
5. Merge the PR

## Build an Example Project Capable of Meeting the Expected Outcome

1. Create a New WPILib Project<br>
    a. by using one of the WPILib 'command' based examples<br>
        i. Select a project type -> Template <br>
        ii. Select a language -> java <br>
        iii. Select a project base -> Command Robot <br>
    b. select project directory - e.g., %root%/camp/username<br>
    c. Name your project with something that reflects the expected outcome<br>
    d. Team Number - 4682<br>
    e. Click Generate Project<br>
2. Import External Packages
3. Research expected APIs that will be needed
4. Add appropriate command classes
5. Add appropriate subsystem classes
6. Complie
7. Deploy
8. Test
