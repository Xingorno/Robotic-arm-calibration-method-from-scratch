# Local_and_Global Calibration Method (for robotic arms)
Objective: this method is to help calibrate the robotic arms, which is a general calibration method that can be used to commercial robots and prototype robots.

## Method
- Step 1: If encoders of your robotic arm need to be calibrated firstly, which mostly happens in lab environments, you can use our lookup table method to calibrate your encoders.
Our way is kind of time-consuming. (If you or your lab doesn't use decent encoders or doesn't have a professional gauge to calibrate encoders, you can try our way.)

- Step 2: After finishing the step 1, we can move onto the robotic arm calibration step. We used a Levenberg-Marquart-based nonlinear regression method to find the optimal link lengths, joint angle offsets and joint axis offsets. (Note: our method requires enough data to train those calibration parameters. For example, in our case, we collected 300 points to optimize 14 parameters, which can get quite decent results.)

## Configuration
- Coding in Matlab
- Optical tracking system (ground truth). You can use other precise measuring systems to get the ground truth data as well.

