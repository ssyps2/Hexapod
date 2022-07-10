clear;
clc;

cmd_spd = [0.001,0,0];   % x, y, z cmd speed
angle_base2leg = [atan2(6,12),pi/2,pi-atan2(6,12),-atan2(6,12),-pi/2,atan2(6,12)-pi];  % related to z-axis, leg1~6
length_base2leg = 0.18;  % unit:m

theta_zero = [0,0,0];
theta_stand = [0,deg2rad(-12),deg2rad(-74)];   % should be the feedback angle by servo in rad
theta_front = [deg2rad(-41),deg2rad(-22),deg2rad(-100)];
theta_back = [deg2rad(7.2),deg2rad(2.4),deg2rad(-43)];
theta_up = [deg2rad(-24),deg2rad(-60),deg2rad(-117)];

theta = theta_stand;

%              theta(i)  offset(i)  a(i-1)  alpha(i-1)
modified_DH = [theta(1,1)   0        0         0;
               theta(1,2)   0        0.043   -pi/2;
               theta(1,3)   0        0.073     pi];   % length of link3 is 0.133m

%             theta(i)   offset(i)  a(i)  alpha(i)
standard_DH = [theta(1,1)   0      0.043   -pi/2;
               theta(1,2)   0      0.073     pi;
               theta(1,3)   0      0.133     0];

%% matrix transformation for one leg
%  rotx/y/z would related to fixed angle, so that:
%  euler angle: z->y->x => fixed angle: x->y->z, they're equivalent
%  for the leg, in euler angle: z1->x1->z2->z3
R0_1 = [rotz(theta(1,1)),[0;0;0]; 0,0,0,1];
R1_2 = [rotx(standard_DH(1,4))*rotz(theta(1,2)),[0;0;0]; 0,0,0,1];
R2_3 = [rotx(standard_DH(2,4))*rotz(theta(1,3)),[0;0;0]; 0,0,0,1];

T0_1 = [eye(3,3),[0.043; 0; 0]; 0,0,0,1];
T1_2 = [eye(3,3),[0.073; 0; 0]; 0,0,0,1];
T2_3 = [eye(3,3),[0.133; 0; 0]; 0,0,0,1];

HTM_leg = R0_1 * T0_1 * R1_2 * T1_2 * R2_3 * T2_3;
HTM_base2leg = cell(6,1);

for i = 1:6
    HTM_base2leg{i} = [rotz(angle_base2leg(1,i)),[0;0;0]; 0,0,0,1] * transl([length_base2leg;0;0]) * HTM_leg;
end

%% robot kinematics model
L1 = Link(standard_DH(1,:),'standard');  % length in m, revolute joint
L2 = Link(standard_DH(2,:),'standard');
L3 = Link(standard_DH(3,:),'standard');

% robot_leg = cell(6,1);
robot_leg = SerialLink([L1,L2,L3],'name','leg3');

start_pos = robot_leg.fkine(theta);
% pace = 0.1;    % unit:m
% start_pos.t(1,1) = start_pos.t(1,1) + pace;
end_pos = robot_leg.fkine(theta_front);
q1 = robot_leg.ikine(start_pos,'mask',[1,1,1,0,0,0]);
q2 = robot_leg.ikine(end_pos,'mask',[1,1,1,0,0,0]);

sample_dot = 20;
[q ,qd, qdd] = jtraj(q1,q2,sample_dot); % 五次多项式轨迹，得到关节角度，角速度，角加速度，20为采样点个数

grid on
end_trial = robot_leg.fkine(q);%根据插值，得到末端执行器位姿
% plot3(end_trial(:).t(1,1),end_trial(:).t(1,2),end_trial(:).t(1,3));%输出末端轨迹
hold on

robot_leg.plot(q);
% robot_leg.teach();
