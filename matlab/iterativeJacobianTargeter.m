
% calculate the pseudo-inverse kinematics using the iterative jacobian method
% no analytical solution exists (I assume but didn't try) so need to find iteratively

% note that deltaX must be small for this to converge

% from Homework 5 problem 5


clear

%% set up state
% fixed joints
state.a2 = 0; % joint width, meters
state.d6 = 0; % wrist length, meters
state.a3 = 10;
state.a4 = 10;
state.a5 = 20;

% variable joints
state.d1 = 50;
state.theta2 = 0;
state.theta3 = pi/2;
state.d4 = 50;
state.theta5 = 0;
state.theta6 = 0;
state.theta7 = 0;

% ship joints
state.ship_yaw = 0;
state.ship_pitch = 0;
state.ship_roll = 0;
state.arm_x = 1;
state.arm_y = 3;
jointRateLimits = [0.25; 0.1; 0.1; 0.25; 0.1; 0.1; 0.1];


state = dhParameters(state);

global pT
pT = [0; 50; -12]; % point tool tip relative to origin 7

[J, J_tran] = Jacobian(state,pT);


initialJointPositions = forwardKinematics(state, pT, true);




dt = 0.5;

% PGoalBase = [1.5; 1; 2.75];
% PGoal = PGoalBase;
PGoal = [50; 0; 75];

for steps = 1:50
    
    %PGoal = PGoal + 4*[(rand(1)-0.5); (rand(1)-0.5); (rand(1)-0.5)];
    
    stateGoal = iterativeJacobian(state, PGoal);
    state = takeStep(state, stateGoal, jointRateLimits, dt);
    
    newJointPositions = forwardKinematics(state, pT, true);
    
    tipPoints(steps,1:3) = newJointPositions(:,end);
    
    
    % pull out the parameters that we want
    time(steps,1) = dt*(steps-1); % need time!
    d1(steps,1) = state.d_i(1);
    theta2(steps,1) = state.theta_i(2);
    theta3(steps,1) = state.theta_i(3);
    d4(steps,1) = state.d_i(4);
    theta5(steps,1) = state.theta_i(5);
    theta6(steps,1) = state.theta_i(6);
    theta7(steps,1) = state.theta_i(7);   
    
    
    plot3(initialJointPositions(1,:), initialJointPositions(2,:), initialJointPositions(3,:));
    hold on
    plot3(newJointPositions(1,:), newJointPositions(2,:), newJointPositions(3,:), 'r');
    plot3(PGoal(1),PGoal(2),PGoal(3), 'r*')
    xlabel('x')
    ylabel('y')
    axis('equal')

end


T = table(time, d1, theta2, theta3, d4, theta5, theta6, theta7);

makeRoboWorksAnimation(T, 'animationTest1');




function newState = takeStep(state, stateGoal, jointRateLimits, dt)
% just linearly interpolate the joint positions?

q = dh2q(state);
qGoal = dh2q(stateGoal);
qNew = zeros(size(q));

qDeltaTotal = qGoal - q;
qDeltaAllowable = jointRateLimits * dt;

for n = 1:length(qDeltaTotal)
   
    if abs(qDeltaTotal(n)) <= qDeltaAllowable(n)
        qNew(n) = qGoal(n); % are allowed to move this much this time step
    else
        qNew(n) = q(n) + qDeltaAllowable(n) * sign(qDeltaTotal(n)); % move in the wanted direction by the allowable delta       
    end
    
end

newState = q2state(qNew, state);

end




function [newState] = iterativeJacobian(state, PGoal)

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

for iter = 1:4
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
    
    q(5) = pi/2 - q(3);
    q(6) = 0; % this is the angle of the tip, want it to remain horizontal?
    
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