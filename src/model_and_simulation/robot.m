%% parameter order: link length, link twist, link offset, joint angle
clear;
clc;

theta=[0.5,0.8,1.1];     % should be the feedback angle by servo

%             a(i-1)  alpha(i-1)  offset(i)  theta(i)
Modified_DH = [0           0	    0   	 theta(1,1);
               0.043	 -pi/2      0        theta(1,2);
               0.073	   pi  	    0   	 theta(1,3)];   % length of link3 is 0.133m

%             a(i)   alpha(i)  offset(i)  theta(i)
Standard_DH = [0.043   -pi/2	   0   	  theta(1,1);
               0.073	 pi        0      theta(1,2);
               0.133	 0   	   0   	  theta(1,3)];

hexa = rigidBodyTree;

bodies = cell(3,1);
joints = cell(3,1);
for i = 1:3
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},Standard_DH(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(hexa,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(hexa,bodies{i},bodies{i-1}.Name)
    end
end

ik = inverseKinematics('RigidBodyTree',hexa);

show(hexa);
