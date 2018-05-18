# Model Predictive Control Project

### Model
The Model that I used is exactly the simpe Kinematic Bicycle Model. Although it is not as accurate as more complex dynamic models, it is a good enough estimation for most of vehicles with different mass. The best advantage of this model is that we can run it in real time.
This model includes the x, y position of the vehicle, as well as orientation angle, velocity, cross-track error and error from the desired heading angle.
In each timestep, the next state of all these variables will be estimated by MPC. 

```
state = [x, y, v, cte, epsi]; 
actuators = [steering_angle, acceleration_input(thorttole)]
```
Following equations are used to estimate the next state at time t+1:

```
next_state[0] = state[0] + state[3] * cos(state[2])*dt;
next_state[1] = state[1] + state[3] * sin(state[2])*dt;
next_state[2] = state[2] + state[3]/Lf *actuators[0]*dt; 
next_state[3] = state[3] + actuators[1]*dt;
```
Steering angle input is a multiplicative factor in estimating the next heading angle in future. Accelaration input also is considered in 
estimation of velocity at time t+1.

### Timestep Length and Elapsed Duration
Since we are trying to run the model in real time, N shouldn't be too long. I picked 12 for the Elapsed Duration. 
Considering the 100 milisecond delay applied to our model, I decided to pick a timestep slightly longer than 100 msec delay to compensate for it. As a result the designed MPC could perform very well in handling this latency.

### MPC Preprocessing
x and y of waypoints are outputed by the simulator, in the global coordinate system. However the position and orientation of the car are in reference to the vehicle's coordinate system.  Also the orientation of the car is presented in standard format and we need to convert it back to Unity format.
I used the suggested trick in QA video to shift the orientation of the car back to Unity format. This would also make it easier to fit polynimial to the waypoints and calculating the cross track error and psi error.

## MPC Tunning
To optimize the MPC for the best results, following items are implemented in cost function:
1. Cross track error and Car's heading angle error are two most important errors to optimize in this project, so they have the heaviest 
2. Cost weights in the cost function.
3. To acheive smoother actuator inputs, there are some costs for the rate of change in the inputs in two consecutive timesteps.
4. To force the car go as fast as reference velocity of 100 mph, there is a small weight on velocifty error cost function as well. 


