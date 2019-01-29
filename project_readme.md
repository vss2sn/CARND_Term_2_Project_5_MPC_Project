## CARND_Term_2_Project_5_MPC_Project

### Model

The MPC model uses state variables (specified below) and provides actuator variables as outputs. Optimal actuator variables are selected by minimizing the cost function under constraints defined by a kinematic model for motion.

#### States

I use the following states as MPC input:

1. (x, y): Current car's position in map coordinate.
2. psi: Current car heading angle from x axis. Rounding counterclockwise means increasing psi.
3. v: Current speed of the car.
4. cte: Cross track error.
5. epsi: Difference between designed angle psi des (road orientation) and current car's heading angle.

#### Actuators

1. delta: steering angle.
2. a: the throttle of the car which normalized to [-1, 1].

#### Update equations
Update equations are used as constraint functions. In this project, the following kinematic model is used (as in the lectures):

x_t+1 = x_t + v_t\*cos(psi_t)\*dt
y_t+1 = y_t + v_t\*sin(psi_t)\*dt
psi_t+1 = psi_t +v_t/L_f delta_t dt
v_t+1 = v_t + a_t\*dt
cte_t+1 =  f(x_t) - y_t + v_t\*sin(epsi_t)\*dt
epsi_t+1 = psi_t - psi des_t + v_t/L_f delta_t dt,

where L_f is the distance between the front of the vehicle and its center of gravity, v_t is v at time t, andf(x_t) represents the function of reference trajectory which is obtained by applying polynomial fitting to the given waypoints.

### Cost function
I was interested in seeing how this cost function might be modified and played around with it.
A good implementation was found here:
// REF: https://github.com/i-aki-y/CarND-MPC-Project
I have used these weights directly.

## Timestep Length and Elapsed Duration (N & dt)
The optimizer predicts future car states up to time T according to a given update equation (in discrete time) and uses these states to evaluate the best actuators.

The time T is defined by two hyper-parameters T = n * dt, where n is a number of prediction step and dt is a time interval between two consecutive predictions.

Assuming T constant, an increase of n will improve the accuracy; however a large n requires more calculation cost. Therefore, there is a trade-off between accuracy and calculation cost. If T is too small, the predicted path is too short; if it is too large then points further ahead are given excessive importance at the cost of the ones closer to the car.

After testing, the chosen values are:
N = 8;
dt = 0.15;

Other values tested include:
| N | dt |
| 8 | 0.15 |
| 10  | 0.10  |
| 10  | 0.15  |
| 12  | 0.15  |

Some of the other values also give decent results.

## Polynomial Fitting and MPC Preprocessing
As suggested in the videos, a 3 degree polynomial function fitting is applied to the given waypoints.

f(x) = c_3 x^3 + c_2 x^2 + c_1 x + c_0.

// NOTE: Considering restricting the length of the predicted model to the first root of df/dx so the curve does not have multiple turns. Realistically, a car on the road without any other cars to overtake will always move in a straight line or a monotonically increasing/decreasing curve in the very short term.

Approximation by using the function f(x) will fail when the road close to vertical angle. To keep the approximation applicable the coordinates need to be transformed from map coordinates to car coordinates using:

x_p' =  (x_p - x_c)cos(-psi) - sin(-psi)
y_p' =  (y_p - y_c)sin(-psi) + cos(-psi),

Hence, cte_t and epsi are obtained by the following equations:
cte_t = f(x=0) - y_c = c_0,
epsi_t = tan^-1(f'(x=0)) - psi = tan^-1(c_1)

## Model Predictive Control with Latency

The existence of latency causes the mismatch between the car states in which the optimal actuators are derived and the states actually performed the derived actuators. Adjusting and accounting for this latency time when implementing the motion model gives good results.

x_pred = x_t + v_t cos(psi_t) dt
y_pred = y_t + v_t sin(psi_t) dt
psi_pred = psi_t + v_t/L_f delta_t dt
v_pred = v_t + a_t dt
cte_pred =  f(x_pred) - y_pred
epsi_pred = psi_pred - psi des_pred
