# Local_and_Global Calibration Method (for robotic arms)
Objective: this method is to help calibrate the robotic arms, which a general calibration method that can be used to commercial robots and prototype robots

## Method
- Step 1: if the encoder of your robotic arm needs to be calibrated firstly, which mostly happens in lab environments, you can use our lookup table method to calibrate the encoders firstly. 
This way is kind of time-consuming. If you or your lab doesn't use the decent encoders or doesn't have a professional gauge to calibrate, you can try our way. 

- Step 2: after calibrating the encoders, we can move onto calibrating the robotic arm, which is to help us to find the optimal link lengths, joint angle offsets and joint axis offsets. 

## Configuration
- Matlab
- Optical tracking system (ground truth). You can use other systems to get the ground truth data as well.

