# GantryCraneController
A controller based on LQR and LQG techniques to obtain a suitable response with an optimal input for the gantry crane with two suspended masses. Controller design is validated by testing it against the non-linear model and simulating the response to initial conditions using Simulink

<p align="center">
  <img src="https://github.com/rishabh1b/GantryCraneController/blob/master/finalOutput.gif?raw=true" alt="Response to input of 10 meters of the cart"/>
</p>

*You need MATLAB & Simulink with SimMechanics workbench support. Preferrably a version later than 2015b.*

### Running on your system
1. `git clone https://github.com/rishabh1b/GantryCraneController.git`
2. Launch MATLAB
3. Change to the working directory
4. First load the parameters in `param_final.mat` using `load('param_final.mat')`
5. Then run the file `finalProjectLQGwithIntegralAction.slx` to see the SimMechanics model in action.
