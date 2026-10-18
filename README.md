## Use module run to run scripts

Tips - create a virtual environment

## Robot setup

BEFORE SERVOS ARE ASSEMBLED

To set all servos into the initial position, calibrate servos and map servos

you can run the special web application directly from the robot

to do it, run this command in the terminal - 

`python -m cubebot_setup.server`

it will run the web server and then you can go to the browser from PC and open this application:

'http://YOUT_PI_IP:5057'

### First steps

1 - After you run this web application click to the button 'Set zero positions' - all servos will move to 90 degree position.
Then, move all disassembled legs parts as they are shown in 3d view - and after this you can assemble servos to joints. Keep this parts in the same position as in 3d view! (Note - there can be a little difference, that can be calibrated later)

2 - When you assemle servos, click to 'Load settings' - it will load default indexes from the config file.
Then, try to set angle to each joints through the web application and see that they are synced with the real robot.
If there is different motor is rotated, try to change index near to the progress bar.
Don't forget to check all servos are synced.
After you did all synchronisations, click on 'Save settings' - it will save the config

3 - Each time you run the web application don't forget to load settings, and don't click 'Save settings' if the servos are actually synced.

4 - Calibration. If there is some difference between the 3d view and the real robot, you can calibrate the legs.
To do it, setup all servos to zero position (click this button)
Then, where is the difference, change the joint angle value a bit, while the real leg part will be in the same position at it WAS SHOWN in 3d view (TO-DO - add this calibration through the 3d view better)
Then, remember the difference between the 90 angle and the set angle and write it into calibration.json.
Notes - it will be changed.

5 - Now the servos are ready


### Notes

You can find files for servo board in pwm_servo/build/pwm_servo.uf2
