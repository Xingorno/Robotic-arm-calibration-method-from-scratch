# Local-and-Global Calibration Method (for robotic arms)
This method is to calibrate the robotic arms. Our approach is a generic calibration method, which can be used for both commercial robots and prototype robots. This figure (please see below) shows our developed robotic 3D ultrasound system. If the architecture of your system is different from ours, please see below to learn how to modify the code to adapt to your system.


# Citation
If you want to use our code for your research, please cite our code.
```
@inproceedings{xing20222d,
  title={A 2D/3D US/CT-guided system for percutaneous focal liver thermal ablation},
  author={Xing, Shuwei and Cool, Derek W and Gardi, Lori and Bax, Jeffrey and Chen, Elvis CS and Peters, Terry M and Fenster, Aaron},
  booktitle={Medical Imaging 2022: Image-Guided Procedures, Robotic Interventions, and Modeling},
  volume={12034},
  pages={237--245},
  year={2022},
  organization={SPIE}
}
```

## Workflow
- Step 1: If the encoders of your robotic arm need to be calibrated first, which mostly happens in the lab setting, you can use our lookup table method to calibrate your encoders.
Our way is kind of time-consuming. (If you or your lab doesn't use decent encoders or doesn't have a professional gauge to calibrate encoders, you can try our way.)

- Step 2: After finishing step 1, we can move on to the robotic arm calibration step. We used a simplified Levenberg-Marquart-based nonlinear regression method to find the optimal link lengths, joint angle offsets, and joint axis offsets. (Note: our method requires enough data to train those calibration parameters. For example, in our case, we collected 300 points to optimize 14 parameters, which can help us get very decent results.)

## How to use the code
- LiverSystemCalibration_V1/2.m. The main code for calibrating our liver system that can be used as well for other systems. If the robotic arm architecture is different from ours, you might need to change the "SimplifiedCalibrationFunction.m".
- LookupTable.m. This file is to achieve step 1 of the workflow. Most systems will not need this step. 
- ./Data. This folder includes some of our collected data, which you can use to have a quick test of our code.
- ./Gyn_system. This folder is for another robotic arm, which can be helpful for you to learn how to modify the code to adapt to your robotic arms.
- 
## Configuration
- Matlab. The version is quite flexible. We used Matlab 2023b to develop.
- [NDI Polaris](https://www.ndigital.com/optical-navigation-technology/polaris-vega-vt/) Optical tracking system. You can use other precise measuring systems to get the ground truth data as well.


