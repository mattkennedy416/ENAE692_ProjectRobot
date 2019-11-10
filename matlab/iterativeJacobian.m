
% calculate the pseudo-inverse kinematics using the iterative jacobian method
% no analytical solution exists (I assume but didn't try) so need to find iteratively

% note that deltaX must be small for this to converge

% from Homework 5 problem 5




%% set up state
% fixed joints
state.a2 = 0; % joint width, meters
state.d6 = 0.25; % wrist length, meters

% variable joints
state.d1 = 2;
state.theta2 = 0;
state.theta3 = pi/2 + pi/4;
state.d4 = 2.5;
state.theta5 = -pi/4;
state.theta6 = 0;
state.theta7 = 0;

dh = dhParameters(state);

pT = [1; 0; -0.25]; % point tool tip relative to origin 7

[J, J_tran] = Jacobian(dh,pT);


initialJointPositions = forwardKinematics(dh, pT, true);


q_initial = dh2q(dh); % get the initial state params
q = q_initial;

state_temp = dh;

PGoal = [1.5; 0; 2.75];

for iter = 1:10
    q_initial = q;
    
    jointPositions = forwardKinematics(state_temp, pT);
    x_initial = jointPositions(:,8);
    
    fprintf("LOOP %i =>  theta = (%f,%f,%f)\t\t p = (%f,%f,%f)\n", iter, q(1),q(2),q(3),x_initial(1),x_initial(2),x_initial(3));
    deltaX = PGoal - x_initial;
    
    % if DOF > 6 then use the pseudoinverse Jacobian!
    [~, J_tran] = Jacobian(state_temp, pT);
    Jpi = pseudoInverseJacobian(J_tran);
    
%     deltaQ = J_tran\deltaX; % equiv to inv(J)*deltaX
    deltaQ = Jpi * deltaX;
    q = q_initial + deltaQ;
    
    state_temp = q2state(q, state_temp); % update the state
end
dh = state_temp;
newJointPositions = forwardKinematics(dh, pT, true);

plot3(initialJointPositions(1,:), initialJointPositions(2,:), initialJointPositions(3,:));
hold on
plot3(newJointPositions(1,:), newJointPositions(2,:), newJointPositions(3,:), 'r');


% height = 5;
% [X,Y,Z] = cylinder(0.2);
% Z = Z*height;
% 
% surf(X,Y,Z)





function Jpi = pseudoInverseJacobian(J)
% if DOF > 6 then use the pseudoinverse Jacobian 

Jpi = transpose(J) / ( J * transpose(J) );

end


function jointPositions = forwardKinematics(dh, pT, includeOrigin)
% need to get the updated position for the new q

jointPositions = zeros(3,size(dh,1));
T_total = eye(4);
for link = 1:size(dh,1)
   T_total = T_total * Transform( dh.theta_i(link), dh.alpha_i1(link), dh.a_i1(link), dh.d_i(link) );
   jointPositions(:,link) = transpose(T_total(1:3,4));
end

% and then to get the tool tip at the end
% pT(3) becomes the d and pT(1) becomes the a? 
T_total = T_total * Transform( 0, 0, pT(3), pT(1) );
jointPositions(:,link+1) = transpose(T_total(1:3,4));

if nargin == 3
    jointPositions = cat(2, [0;0;0], jointPositions);
end


end


function q = dh2q(dh)
% assemble q from the variable parameters

q = zeros(7,1);
q(1) = dh.d_i(1);
q(2) = dh.theta_i(2);
q(3) = dh.theta_i(3);
q(4) = dh.d_i(4);
q(5) = dh.theta_i(5);
q(6) = dh.theta_i(6);
q(7) = dh.theta_i(7);

end

function state = q2state(q, state)
% insert an updated q back into the state table

state.d_i(1) = q(1);
state.theta_i(2) = q(2);
state.theta_i(3) = q(3);
state.d_i(4) = q(4);
state.theta_i(5) = q(5);
state.theta_i(6) = q(6);
state.theta_i(7) = q(7);

end




function T_ii1 = Transform(theta_i, alpha_i1, a_i1, d_i)

T_ii1 = [cos(theta_i), -sin(theta_i), 0, a_i1;
    sin(theta_i)*cos(alpha_i1), cos(theta_i)*cos(alpha_i1), -sin(alpha_i1), -sin(alpha_i1)*d_i;
    sin(theta_i)*sin(alpha_i1), cos(theta_i)*sin(alpha_i1), cos(alpha_i1), cos(alpha_i1)*d_i;
    0, 0, 0, 1];

end