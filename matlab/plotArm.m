

jointPositions = forwardKinematics(dh, pT, true);


% armRadius = 0.2;
% 
% 
% % first arm stretch
% [X,Y,Z] = cylinder(armRadius);
% Z = Z*jointPositions(3,2);
% surf(X,Y,Z)
% 
% hold on
% 
% 
% 
% 
% [X,Y,Z] = cylinder(armRadius);


% this may work, but also we can just plot it as lines ... don't need to
% complicate things with 3d

plot3(jointPositions(1,:), jointPositions(2,:), jointPositions(3,:), 'r');

