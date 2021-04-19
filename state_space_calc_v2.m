
g = 9.816; %Gravitation acceleration

H = 0.144; %Body height

M = 0.600; %Body mass
L = H/2; %Body length to center of mass
W = 0.14; %Body width
D = 0.04;

R = 0.04; %Wheel radius
n = 1; %Gear ratio
m = 0.03; %Wheel weight

J_psi = M*(L^2)/3; %Body pitch inertial moment
J_theta = M*(W^2 + D^2)/12; %Body yaw inertial moment
J_m = 1e-5; %DC motor inertial moment
J_w = m*(R^2)/2; %Wheel inertial moment

f_m = 0.0022; %Friction coefficient between body and DC motor
f_w = 0; %Friction coefficient between wheel and floor.

K_b = 0.468; %DC motor back EMF constant #TODO
K_t = 0.317; %DC motor torque constant
R_m = 6.69; %Motor resistance

alpha = n*K_t/R_m;
beta = n*K_t*K_b/R_m + f_m; 

%%%%%%%%%%%%%%%%%%%%%%

I = (m * W^2)/2 + J_theta + (W^2)*(J_psi + J_m * n^2)/(2 * R^2);
J = (W^2)*(beta + f_w)/(2*R^2);
K = W*alpha/(2*R);

S = zeros(2);
S(1,1) = (2*m + M)*(R^2)+2*J_w+2*(n^2)*J_m;
S(1,2) = M*L*R-2*(n^2)*J_m;
S(2,1) = M*L*R - 2*(n^2)*J_m;
S(2,2) = M*(L^2)+J_psi+2*(n^2)*J_m;

T = zeros(2);
T(1,1) = 2*(beta + f_w);
T(1,2) = -2*beta;
T(2,1) = -2*beta;
T(2,2) = 2*beta;

U = zeros(2);
U(2,2) = -2*M*g*L;

V = zeros(2);
V(1,1) = 2*alpha;
V(1,2) = 2*alpha;
V(2,1) = -2*alpha;
V(2,2) = -2*alpha;

A = [zeros(2) eye(2);-(S\U) -(S\T)];
B = [zeros(2); S\V];

sys = ss(A, B, eye(4), 0);
Ts = 0.03;
sysd = c2d(sys, Ts);

A_d = sysd.A;
B_d = sysd.B;

poles = [0.55+0.1*i, 0.55-0.1*i, 0.40+0.05*i, 0.40-0.05*i];
K = place(A_d, B_d, poles)

