

clear


%% set up state
% fixed joints
state.a2 = 0; % joint width, meters
state.a3 = 10;
state.a4 = 10;

state.d6 = 20; % wrist length, meters
state.a5 = 0; % wrist length? 

global pT
pT = [0; 50; -12]; % point tool tip relative to origin 7


% % variable joints - default state hanging out over ship
state.d1 = 50;
state.theta2 = 0;
state.theta3 = pi/2;
state.d4 = 50;
state.theta5 = 0;
state.theta6 = 0;
state.theta7 = 0;


jointRateLimits = [1; 0.1; 0.1; 1; 0.1; 0.1; 0.1];


state = dhParameters(state);


%% run the iterative jacobian

initialJointPositions = forwardKinematics(state, pT, true);

targetPosition = [65; 65; 65;];
targetPitch = -pi/8;
targetYaw = pi/8;


intermediateJointPositions = initialJointPositions;
for n = 1:25

    P5T = intermediateJointPositions(:,9) - intermediateJointPositions(:,6);
    
    PGoal = targetPosition - P5T(1:3);

    targetOrigin = 5;

    % so actually our goal value is going to be a function of the goal? need to
    % solve iteratively? ie the angle of tip relative to joint 5 is a function
    % of where joint 5 is
    [stateGoal, ~] = iterativeJacobian(state, PGoal, targetOrigin);
    stateGoal.theta_i(5) = pi/2 - stateGoal.theta_i(3);
    stateGoal.theta_i(6) = targetPitch;
    stateGoal.theta_i(7) = targetYaw - stateGoal.theta_i(2);
    intermediateJointPositions = forwardKinematics(stateGoal, pT, true);
    
    if norm(intermediateJointPositions(:,9) - targetPosition) < 1E-2
        break
    end

end



[stateGoal, iterativeJacobianError] = iterativeJacobian(state, PGoal, targetOrigin);

stateGoal.theta_i(5) = pi/2 - stateGoal.theta_i(3);
stateGoal.theta_i(6) = targetPitch;
stateGoal.theta_i(7) = targetYaw - stateGoal.theta_i(2);

q = dh2q(state);
qGoal = dh2q(stateGoal);


finalJointPositions = forwardKinematics(stateGoal, pT, true);

steps = 1;
d1(steps,1) = state.d_i(1);
theta2(steps,1) = state.theta_i(2);
theta3(steps,1) = state.theta_i(3);
d4(steps,1) = state.d_i(4);
theta5(steps,1) = state.theta_i(5);
theta6(steps,1) = state.theta_i(6);
theta7(steps,1) = state.theta_i(7); 

steps = 2;
d1(steps,1) = stateGoal.d_i(1);
theta2(steps,1) = stateGoal.theta_i(2);
theta3(steps,1) = stateGoal.theta_i(3);
d4(steps,1) = stateGoal.d_i(4);
theta5(steps,1) = stateGoal.theta_i(5);
theta6(steps,1) = stateGoal.theta_i(6);
theta7(steps,1) = stateGoal.theta_i(7); 

time = [0; 1];

armState = table(time, d1, theta2, theta3, d4, theta5, theta6, theta7);
makeRoboWorksAnimation(armState, 'inverseKinematicsExample');



function [newState, error] = iterativeJacobian(state, PGoal, targetOrigin)

global pT

% I think we want to stabalize the tip, so should we be calculating the
% optimization error based off of a fixed position from joint 4?
% and then separately calculate 5, 6, and 7? well no because we want 6 and
% 7 flexible
% really we just want 3 and 5 to be functions of each-other
% can we just manually update 5?
% want theta3 + theta5 = 90, so theta5 = pi/2 - theta3?

% ------------------------------------

% alright so we're actually going to split the arm and wrist controllers,
% working backwards:
% - define position, pitch, and yaw of the target (from the drone position)
% - calculate a wrist roll such that the catcher is flat
% - this should give us a required position for origin 5 (or maybe 4? are they the same)
% - run iterative jacobian to solve


q_initial = dh2q(state);
q = q_initial;

state_temp = state;

for iter = 1:10
    q_initial = q;
    
    jointPositions = forwardKinematics(state_temp, pT);
    x_initial = jointPositions(:,targetOrigin);
    
    fprintf("LOOP %i =>  theta = (%f,%f,%f)\t\t p = (%f,%f,%f)\n", iter, q(1),q(2),q(3),x_initial(1),x_initial(2),x_initial(3));
    deltaX = PGoal - x_initial;
    
    % if DOF > 6 then use the pseudoinverse Jacobian!
%     [~, J_tran] = Jacobian(state_temp, pT);
    J_tran = J15_translation(state_temp);
    Jpi = pseudoInverseJacobian(J_tran);
    
%     deltaQ = J_tran\deltaX; % equiv to inv(J)*deltaX
    deltaQ = Jpi * deltaX;
    deltaQ(6:7) = [0;0];
    q = q_initial + deltaQ;
    
%     q(5) = pi/2 - q(3);
%     q(6) = 0; % this is the angle of the tip, want it to remain horizontal?
    error(iter) = norm(deltaQ);
    state_temp = q2state(q, state_temp); % update the state
end

newState = state_temp;


end








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
   jointPositions(:,link) = T_total(1:3,4);
end

% and then to get the tool tip at the end
% pT(3) becomes the d and pT(1) becomes the a? 
T_total = T_total * Transform( 0, 0, pT(3), pT(2) );
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