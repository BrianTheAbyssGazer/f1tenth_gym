function f = vehicleDynamics_KSP(x,p,ref,ki1,kp1,kd1,ki2,kp2,kd2)
% vehicleDynamics_KST - kinematic single-track with on-axle trailer 
% reference point: rear axle
% Inputs:
%    x - vehicle state vector
%    u - vehicle input vector
%    p - vehicle parameter structure
%------------- BEGIN CODE --------------

%create equivalent kinematic single-track parameters
l = p.a + p.b;

%states
%x1 = s_x x-position in a global coordinate system
%x2 = s_y y-position in a global coordinate system
%x3 = δ steering angle of front wheels
%x4 = u velocity in x-direction
%x5 = Ψ yaw angle

%inputs
%u1 = v_delta steering angle velocity of front wheels
%u2 = ax longitudinal acceleration
%control input
x_bar=ref - [x(1) x(2)]; % vector from current loctaion to reference location
v_yaw=[cos(x(5)) sin(x(5))]; % unit vector along pose direction
v_lateral=[cos(x(5)) sin(x(5))]; % unit vector along pose direction
min_turning_center_L=[x(1) x(2)]+v_lateral;
min_turning_center_R=[x(1) x(2)]-v_lateral;
R = p.l / tan(p.steering.max);
lateral_distance=abs(dot(v_lateral,x_bar));
if (ref(1)-min_turning_center_L(1))^2+(ref(2)-min_turning_center_L(2))^2<R^2 ||...
    (ref(1)-min_turning_center_R(1))^2+(ref(2)-min_turning_center_R(2))^2<R^2
    kp1=-kp1;
    u2=kp2*norm(x_bar)+kd2*x(4)+ki2*x(6);
else
    logitude_distance=dot(v_yaw,x_bar);
    u2=kp2*logitude_distance+kd2*x(4)+ki2*x(6);
end

angle_erro=atan2(v_yaw(1)*x_bar(2)-v_yaw(2)*x_bar(1),v_yaw(1)*x_bar(1)+v_yaw(2)*x_bar(2));
u1=kp1*angle_erro+kd1*x(3)+ki1*x(7);

%consider steering constraints
u1 = steeringConstraints(x(3),u1,p.steering);
%consider acceleration constraints
u2 = accelerationConstraints(x(4),u2,p.longitudinal);

%system dynamics
f(1,1) = x(4)*cos(x(5));
f(2,1) = x(4)*sin(x(5));
f(3,1) = u1;
f(4,1) = u2;
f(5,1) = x(4)/l*tan(x(3));
f(6,1) = norm(x_bar);
f(7,1) = angle_erro;
%------------- END OF CODE --------------
