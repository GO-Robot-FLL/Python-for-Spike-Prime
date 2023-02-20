# Python-for-Spike-Prime
These are some basic programs that are useful if you are building and programming the LEGO SPIKE PRIME.
We use this code for competing in the FLL ourselves and have received multiple awards for our robot design.
We strongly recommend that you use VS Code for your programming, the instructions for doing so are below.
You can however also paste this into the SPIKE or MINDSTORMS software.

## Programs and Features included
#### Linefollower 
- follows line until specified color/distance is reached
- acceleration/braking
- dynamic PID control for smooth driving
#### Gyrorotation
- turns until specified color/angle is reached
- acceleration/braking
- different turning styles (on the spot/curve)
#### Gyrostraightdrive
- drives in a straight line until specified distance/color is reached
- accelerating/braking
- dynamic PID control for smooth driving
- alignment with lines
- parallel code execution for turning motors while driving
#### Arc Rotation
- turns in a large arc, to be able to turn around objects until ending condition is reached
- accelerating breaking
- various ending conditions
#### Break Function
- allows you to break out of currently running function back into main
- press right button to activat
- save significant amounts of time and is less prone to crashing the hub than middle button


## Necessary extensions/Programs for use in VS Code:
Visual Studio Code
Python
Lego Spike Prime/Mindstorms Robot Inventor Extension by Peter Staev

## Fully charged battery voltage
When the battery is fully charged the voltage is roughly 8300. This will vary based on how the wear and tear of your battery.
We recommend charging the robot when the battery voltage drops below 8000 mV, as there is a reduction in torque. 

## How to connect:
Click the button *LEGO Hub: Disconnected* on the left hand side of the blue bottom bar. 
A dropdown menu will open at the top of the screen, try out all the COM options it gives you until you are connected.
Note: You have to have previously connected to the hub in the Spike or Mindstorms software first.

## How to upload a program to the hub:
Press CTRL SHIFT P and select *LEGO Hub: upload program* from the dropdown menu.
Choose Python: regular.
Choose the slot you want the program to be saved in on the hub.

## How to start a program:
Press CTRL + SHIFT + P and select *LEGO Hub: start program* from the dropdown menu.
Select the slot of the program you want to start on the hub.
Note: This step can be skipped if you have autostart activated.

## How to enable precompiling:
To precompile the main.py on the PC/laptop, go to the extension page and select the extension.
Then click on the settings symbol and activate Lego Spike Prime Mindstorms: *Compile Before Upload*.
This will allow the program to start faster.

## Reporting Bugs:
If you find a bug, kindly report it to go.robot@gmynasium-ottobrunn.de 
We will try to fix it as soon as possible. If you have any questions regarding the code, please reach out to us using the same Email adress.

## For further info on working with Spike Prime in VS Code see:
https://github.com/PeterStaev/lego-spikeprime-mindstorms-vscode

## Credits:
Thanks to [Peter Staev](https://github.com/PeterStaev/lego-spikeprime-mindstorms-vscode) for making it possible for us to use access in depth features of Spike Prime and for enabling us to use VS Code to program.

## Disclaimer:
This program uses features of extensions that use unofficial and undocumented APIs. They can change without notice. Functions tested on Windows 10/11 connecting to a LEGO MINDSTORMS Spike Prime Hub.

*LEGO* and *MINDSTORMS* are registered trademarks of the LEGO Group. *SPIKE* is a trademark of LEGO Group.
