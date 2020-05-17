%% MECA 482 Project
%%
clc
clear
% State Space Representation
Jp=.8;Mp=.127;Lr=.216;Jr=.8;Lp=.337;g=9.81;Br=37.3;Bp=2;Kg=.9;kt=0.00768;km=.03;...
    Rm=2.6;Mr=.125; %Random Parameters
Jt=Jp*Mp*Lr^2+Jr*Jp+(1/4)*Jr*Mp*Lp^2;
A=[0 0 1 0;0 0 0 1;0 (1/(4*Jt))*Mp^2*Lp^2*Lr*g (-1/Jt)*Br*(Jp+1/4*Mp*Lp^2) (-1/(2*Jt))*Bp*Mp*Lp*Lr;0 .5*Mp*Lp*g*(Jr+Mp*Lr^2)/Jt (1/(2*Jt))*Mp*Lp*Lr*Br (-1/Jt)*Bp*(Jr+Mp*Lp^2)];
B=[0;0;(1/Jt)*(Jp+.25*Mp*Lp^2);(1/(2*Jt))*Mp*Lp*Lr];
C=eye(2,4);
D=zeros(2,1);
% Actuator Dynamics
B=Kg*kt*B/Rm;
A(3,3)=A(3,3)-Kg^2*kt*km/Rm*B(3);
A(4,3)=A(4,4)-Kg^2*kt*km/Rm*B(4);
states={'theta' 'theta_dot' 'alpha' 'alpha_dot'};
inputs={'u'};
outputs={'theta';'alpha'};

sys_Pendulum = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs) % Open loop system model
sys_tf=tf(sys_Pendulum) % System transfer function

% The requirements of the system
zeta = 0.7;
wn = 4;
p3 = -30; % Desired pole location
p4 = -40; % Desired pole location
% Location of dominant poles along real-axis
sigma = zeta * wn;
% Location of dominant poles along img axis (damped natural freq wn*(1-zeta)^0.5)
wd=wn*(1-zeta)^0.5
% Desired poles (-30 and -40 are given)
poles = [-sigma+j*wd, -sigma-j*wd, p3, p4];
% Find control gain using MATLAB pole-placement command (acker or place)
K = acker(A, B, poles)
mu=2.3;
Ep=Mp*g*Lp; %Potential Energy
Er=Ep;
eta_m=0.90;
eta_g=0.70;
K_amp=2.5 ;  %Amplifier gain
