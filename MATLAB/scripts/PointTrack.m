% Script for testing different vehicle models (see examples in chap. 11 of documentation)
clc;
clear;
close all;
%% set parameters and options
clear
addpath(genpath('../vehiclemodels'))

% load parameters
p = parameters_vehicle2;
g = 9.81; %[m/s^2]

% set options
tStart = 0; %start time
tFinal = 200;

delta0 = 0;
vel0 = 0;
Psi0 = 0;
dotPsi0 = 0;
beta0 = 0;
sy0 = 0;
initialState = [0,sy0,delta0,vel0,Psi0,dotPsi0,beta0]; %initial state for simulation
x0_KSP = init_KSP(initialState); %initial state for kinematic single-track model
x0_ST = init_ST(initialState); %initial state for single-track model
ki1=0.0; % steering integrator
kp1=0.9; % steering force
kd1=-3.3; % steering back lead time
ki2=0.0; % position integrator
kp2=2; % drag
kd2=-2; % brake
ref=[10,100];
%% simulate single point tracking
%simulate KPT
[t_coast_st,x_coast_st] = ode45(getfcn(@vehicleDynamics_KSP,p,ref,ki1,kp1,kd1,ki2,kp2,kd2),[tStart, tFinal],x0_KSP);

%% Plot trace
s=size(x_coast_st);
x_bar=[ones(s(1),1)*ref(1) ones(s(1),1)*ref(2)] - [x_coast_st(1) x_coast_st(2)]; % vector from current loctaion to reference location
v_yaw=[cos(x_coast_st(:,5)) sin(x_coast_st(:,5))]; % unit vector along pose direction
angle_difference=atan2(v_yaw(:,1).*x_bar(:,2)-v_yaw(:,2).*x_bar(:,1),v_yaw(:,1).*x_bar(:,1)+v_yaw(:,2).*x_bar(:,2));
figure
subplot(2,1,1)
plot(t_coast_st,x_coast_st(:,1))
title('x')
subplot(2,1,2)
plot(t_coast_st,x_coast_st(:,2))
title('y')
figure
plot(x_coast_st(:,1),x_coast_st(:,2))
hold on
plot(ref(1),ref(2),'r*')
mi=min([min(x_coast_st(:,1)) min(x_coast_st(:,2))]);
ma=max([max(x_coast_st(:,1)) max(x_coast_st(:,2))]);
xlim([mi ma])
ylim([mi ma])
title('trace')
figure
subplot(3,1,1)
plot(t_coast_st,x_coast_st(:,4))
title('velocity')
subplot(3,1,2)
plot(t_coast_st,x_coast_st(:,3))
title('steering')
subplot(3,1,3)
plot(t_coast_st,angle_difference)
title('angle')
R = p.l / tan(p.steering.max)
%% add input and parameters to ode 
function [handle] = getfcn(fctName,p,ref,ki1,kp1,kd1,ki2,kp2,kd2)
    
    function dxdt = f(t,x)
        dxdt = fctName(x,p,ref,ki1,kp1,kd1,ki2,kp2,kd2);
    end

    handle = @f;
end