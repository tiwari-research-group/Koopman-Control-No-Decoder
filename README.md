# Koopman-Control-No-Decoder

This code is designed to implement the Koopman Operator using the method outlined in our paper 'Computationally Efficient Data-Driven Discovery and Linear Representation of Nonlinear Systems for Control'. 
It implements the dynamics of a pendulum and includes the data generation file, the code to train and predict the dynamics linearly, and a MATLAB code to implement the LQR controller on the newly linearised system. 

All data files have been provided, so that the code can be run straightaway but the data generation and LQR controller generation codes have been provided as well. 

data_generator_control.ipynb file: Data generation code which will automatically save the X,Y,U datasets as .pt files. It is possible to change parameters such as dataset size, simulation time, timestep,
noise, dyanmic properties (gravity and length), as well as the size of the random control input. Changing these parameters will alter the training dataset.

No_Decoder_Pendulum_Noisy.ipynb: Neural network training code and Koopman linear dynamic prediction. This code imports the data from the data generator and sorts it into prespecified batches of a user defined batch size
(highly dependent on GPU capabilities. We used 64GB GPU and ran batch sizes of 4096). The Neural Network architecture is also defined, with the hyperparameters being able to be tuned. The training is ran for a 
pre-specified number of epochs. Linear prediticion of the dynamics is also displayed, with comparison to previous methods and the true nonlinear dynamics with error plots given. Finally a plot of the LQR response is
given based on the LQR controller designed in MATLAB.

Koopman_LQR_Controller.m: This MATLAB script manually imports the discrete time domain K and B matrices from the python script and creates an LQR controller based on these dynamics. It does this in both the discrete 
and continuous time domains and saves the continuous time domain data for plotting within the python script. 

Run the data generation script, then the neural network training script to get your K and B matrices and then the MATLAB file to implement the control system to the dynamics. 
