clear;
clc;

cmd_spd = [0.001,0,0];   % x, y, z cmd speed
theta_zero = [0,0,0];
theta_stand = [0,deg2rad(12),deg2rad(28.8)];   % should be the feedback angle by servo in rad
theta_front = [deg2rad(57.6),deg2rad(21.6),deg2rad(55.2)];
theta_up = [deg2rad(24),deg2rad(60),deg2rad(72)];

%              theta(i)  offset(i)  a(i-1)  alpha(i-1)
Modified_DH = [theta_stand(1,1)   0        0         0;
               theta_stand(1,2)   0        0.043   -pi/2;
               theta_stand(1,3)   0        0.073     0];   % length of link3 is 0.133m

%             theta(i)   offset(i)  a(i)  alpha(i)
Standard_DH = [theta_stand(1,1)   0      0.043   -pi/2;
               theta_stand(1,2)   0      0.073     0;
               theta_stand(1,3)   0      0.133     0];

L1 = Link(Standard_DH(1,:),'standard');  % length in m, revolute joint
L2 = Link(Standard_DH(2,:),'standard');
L3 = Link(Standard_DH(3,:),'standard');

leg4 = SerialLink([L1,L2,L3],'name','leg1');

%% matrix transformation for one leg
%  rotx/y/z would related to fixed angle, so that:
%  euler angle: z->y->x => fixed angle: x->y->z, they're equivalent
R0_1 = rotz(theta_stand(1,1)) * rotx(-90,'deg');    % fixed angle: x->z
R1_2 = rotz(theta_stand(1,2));
R2_3 = rotz(theta_stand(1,3));

D0_1 = [0.043; 0; 0];
D1_2 = [0.073; 0; 0];
D2_3 = [0.133; 0; 0];

H0_1 = [R0_1,D0_1; 0,0,0,1];
H1_2 = [R1_2,D1_2; 0,0,0,1];
H2_3 = [R2_3,D2_3; 0,0,0,1];

% HTM_base_leg1 = 

HTM_shld = H0_1 * H1_2 * H2_3;   % foot relative to the shoulder coordinate

HTM = leg4.fkine(theta_stand);
leg_angle = leg4.ikine(HTM,'mask',[1,1,1,0,0,0]);

start_pos = HTM;
% pace = 0.1;    % unit:cm
% HTM.t(1,1) = HTM.t(1,1) + pace;
end_pos = leg4.fkine(theta_front);
q1 = leg4.ikine(start_pos,'mask',[1,1,1,0,0,0]);
q2 = leg4.ikine(end_pos,'mask',[1,1,1,0,0,0]);

sample_dot = 10;
[q ,qd, qdd] = jtraj(q1,q2,sample_dot); % 五次多项式轨迹，得到关节角度，角速度，角加速度，10为采样点个数

grid on
end_trial = leg4.fkine(q);%根据插值，得到末端执行器位姿
% plot3(end_trial(:).t(1,1),end_trial(:).t(1,2),end_trial(:).t(1,3));%输出末端轨迹
hold on

leg4.plot(q);
% leg1.teach();