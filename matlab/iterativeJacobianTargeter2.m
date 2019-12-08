
% calculate the pseudo-inverse kinematics using the iterative jacobian method
% no analytical solution exists (I assume but didn't try) so need to find iteratively

% note that deltaX must be small for this to converge

% from Homework 5 problem 5


clear








%% set up state
% fixed joints
state.a2 = 0; % joint width, meters
state.a3 = 10;
state.a4 = 10;

state.d6 = 20; % wrist length, meters
state.a5 = 0; % wrist length? 

% % % variable joints - default state hanging out over ship
% state.d1 = 50;
% state.theta2 = 0;
% state.theta3 = pi/2;
% state.d4 = 50;
% state.theta5 = 0;
% state.theta6 = 0;
% state.theta7 = 0;

% stowed state:
state.d1 = 50;
state.theta2 = deg2rad(110);
state.theta3 = deg2rad(90);
state.d4 = 50;
state.theta5 = pi/2 - state.theta3;
state.theta6 = 0;
state.theta7 = -state.theta2;




% ship joints
state.ship_yaw = 0;
state.ship_pitch = 0;
state.ship_roll = 0;
state.arm_x = 1;
state.arm_y = 3;
jointRateLimits = [1; 0.1; 0.1; 1; 0.1; 0.1; 0.1];


state = dhParameters(state);

global pT
pT = [0; 50; -12]; % point tool tip relative to origin 7

[J, J_tran] = Jacobian(state,pT);


initialJointPositions = forwardKinematics(state, pT, true);




dt = 0.5;
robotBasePosition = [360; -450; -50];


% need to define the position, pitch, and yaw of target
% where positive x is out of ship, -y is towards back of ship
targetPosition = [75; 45; 65;];

droneState = calculateDroneDynamics(targetPosition, robotBasePosition, dt);
[droneState, catcherState] = calculateCatcherDynamics(droneState, dt);

% targetYaw = droneState.drone_yaw(end); % =theta7, =0 parallel to ship, where increasing theta7 points us toward ship and negative away
% targetPitch = droneState.drone_pitch(end); % = theta6, =0 parallel to ship, where increasing theta6 points the catcher upward, and decreasing points downward
targetYaw = 0;
targetPitch = 0;







%% CATCH THE DRONE!
% so we want the tool tip to be at target position, with theta6 and theta7
% defined
% and we have pT which is the tip relative to origin 7, so
% p7 = targetPosition - pT;

% and then we need the position at 5? so transformation matrix from 7 to 5
T56 = Transform( state.theta_i(5), state.alpha_i1(5), state.a_i1(5), state.d_i(5) );
T67 = Transform( targetPitch, state.alpha_i1(6), state.a_i1(6), state.d_i(6) );
T78 = Transform( targetYaw, pi/2, pT(2), pT(3) );
T58 = T56 * T67 * T78;


t_targetPosition = targetPosition;
t_targetPosition(4) = 0;

P5T = [T58(3,4); T58(1,4); -T58(2,4)]; % don't mess with this, it works

P5T_noRotation = initialJointPositions(:,9) - initialJointPositions(:,6);

% from stowed state we probably need some intermediate points to make sure
% we converge properly
PGoal1 = [0; 50; 60];
PGoal2 = [25; 35; 65];
PGoal3 = [35; 25; 65];
PGoalFinal = targetPosition - P5T(1:3);

PGoal = [PGoal1, PGoal2, PGoal3, PGoalFinal];
goal = 1;
goalError = 1;


targetOrigin = 5;

for steps = 1:size(droneState,1)
    
    %PGoal = PGoal + 4*[(rand(1)-0.5); (rand(1)-0.5); (rand(1)-0.5)];
    
    stateGoal = iterativeJacobian(state, PGoal(:,goal), targetOrigin);
    stateGoal.theta_i(5) = pi/2 - stateGoal.theta_i(3);
    stateGoal.theta_i(6) = targetPitch;
    stateGoal.theta_i(7) = targetYaw - stateGoal.theta_i(2);
    
    
    state = takeStep(state, stateGoal, jointRateLimits, dt);
    
    newJointPositions = forwardKinematics(state, pT, true);
    
    if norm(newJointPositions(:,targetOrigin) - PGoal(:,goal)) < goalError
        if size(PGoal,2) > goal
            goal = goal + 1;
        end
    end
    
    
    % pull out the parameters that we want
    time(steps,1) = dt*(steps-1); % need time!
    d1(steps,1) = state.d_i(1);
    theta2(steps,1) = state.theta_i(2);
    theta3(steps,1) = state.theta_i(3);
    d4(steps,1) = state.d_i(4);
    theta5(steps,1) = state.theta_i(5);
    theta6(steps,1) = state.theta_i(6);
    theta7(steps,1) = state.theta_i(7);   
    
    
    plot3(initialJointPositions(1,:), -initialJointPositions(2,:), initialJointPositions(3,:));
    hold on
    plot3(newJointPositions(1,:), -newJointPositions(2,:), newJointPositions(3,:), 'r');
    plot3(PGoal(1,goal),-PGoal(2,goal),PGoal(3,goal), 'r*')
    xlabel('x')
    ylabel('y')
    axis('equal')

end

plot3(droneState.x-robotBasePosition(1), droneState.y-robotBasePosition(2), droneState.z-robotBasePosition(3))

catcher = catcherState;

armState = table(time, d1, theta2, theta3, d4, theta5, theta6, theta7, catcher);
combinedCatch = join(armState, droneState);


%% RETURN IT TO DECK!
deckTarget = [0; 50; 25]; % this can be P5
PGoal1 = [25; 50; 65];
PGoal2 = [0; 50; 40];
PGoal = [PGoal1, PGoal2];
goal = 1;
goalError = 1;


targetOrigin = 5;

clear time d1 theta2 theta3 d4 theta5 theta6 theta7

droneX = droneState.x(end);
droneY = droneState.y(end);
droneZ = droneState.z(end);
droneRelPos0 = [droneX; droneY; droneZ] - robotBasePosition;
droneRelPos7 = droneRelPos0 - newJointPositions(:,8);
pitch0 = droneState.drone_pitch(end);
roll0 = droneState.drone_roll(end);

targetYaw = pi/2;

t0 = combinedCatch.time(end);
for steps = 1:100
    
    %PGoal = PGoal + 4*[(rand(1)-0.5); (rand(1)-0.5); (rand(1)-0.5)];
    
    stateGoal = iterativeJacobian(state, PGoal(:,goal), targetOrigin);
    stateGoal.theta_i(5) = pi/2 - stateGoal.theta_i(3);
    stateGoal.theta_i(6) = targetPitch;
%     stateGoal.theta_i(7) = targetYaw - stateGoal.theta_i(2);
    stateGoal.theta_i(7) = 0;
    
    
    state = takeStep(state, stateGoal, jointRateLimits, dt);
    
    newJointPositions = forwardKinematics(state, pT, true);
    
    if norm(newJointPositions(:,targetOrigin) - PGoal(:,goal)) < goalError
        if size(PGoal,2) > goal
            goal = goal + 1;
        else
            
            
        end
    end
      
    
    
    x(steps,1) = newJointPositions(1,8) + droneRelPos7(1) + robotBasePosition(1);
    y(steps,1) = newJointPositions(2,8) + droneRelPos7(2) + robotBasePosition(2);
    z(steps,1) = newJointPositions(3,8) + droneRelPos7(3) + robotBasePosition(3);
    drone_yaw(steps,1) = state.theta_i(2) + state.theta_i(7); % does this need to counter-act 2 and 7?
    drone_pitch(steps,1) = pitch0*(1 - drone_yaw(steps,1)/targetYaw); % approx
    drone_roll(steps,1) = pitch0*drone_yaw(steps,1)/targetYaw;
    
    % pull out the parameters that we want
    time(steps,1) = t0 + dt*(steps-1); % need time!
    d1(steps,1) = state.d_i(1);
    theta2(steps,1) = state.theta_i(2);
    theta3(steps,1) = state.theta_i(3);
    d4(steps,1) = state.d_i(4);
    theta5(steps,1) = state.theta_i(5);
    theta6(steps,1) = state.theta_i(6);
    theta7(steps,1) = state.theta_i(7); 
%     theta7(steps,1) = targetYaw - theta2(steps,1); % lets actually enforce this so the drone doesn't get out of sync
     
    
    plot3(initialJointPositions(1,:), -initialJointPositions(2,:), initialJointPositions(3,:));
    hold on
    plot3(newJointPositions(1,:), -newJointPositions(2,:), newJointPositions(3,:), 'r');
    plot3(PGoal(1),-PGoal(2),PGoal(3), 'r*')
    xlabel('x')
    ylabel('y')
    axis('equal')

end

catcher = catcher(end) * ones(length(time),1);

armStateReturn = table(time, d1, theta2, theta3, d4, theta5, theta6, theta7, catcher);
droneStateReturn = table(time, x,y,z,drone_pitch, drone_roll, drone_yaw);

combinedReturn = join(armStateReturn, droneStateReturn);




combinedState = [combinedCatch; combinedReturn];
makeRoboWorksAnimation(combinedState, 'animationTest2');




function newState = takeStep(state, stateGoal, jointRateLimits, dt)
% just linearly interpolate the joint positions?

q = dh2q(state);
qGoal = dh2q(stateGoal);
qNew = zeros(size(q));

qDeltaTotal = qGoal - q;
qDeltaAllowable = jointRateLimits * dt;
stepsAtMaxRate = ceil(abs(qDeltaTotal ./ qDeltaAllowable));

% for the wrist at least, lets get all the joints to move at a rate such
% that they'll all finish at the same time
% and actually lets include joint 3 in there as well since it's tied to 5
% wristSteps = max(stepsAtMaxRate([3,5,6,7]));
% 
% for n = [3,5,6,7]
%    qNew(n) = q(n) +  qDeltaTotal(n)/wristSteps;
% end
% this actually looks even worse... meh

% and then leave the rest of the joints as-is?
for n = 1:7
   
    if abs(qDeltaTotal(n)) <= qDeltaAllowable(n)
        qNew(n) = qGoal(n); % are allowed to move this much this time step
    else
        qNew(n) = q(n) + qDeltaAllowable(n) * sign(qDeltaTotal(n)); % move in the wanted direction by the allowable delta       
    end
    
end

newState = q2state(qNew, state);

end




function [newState] = iterativeJacobian(state, PGoal, targetOrigin)

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

for iter = 1:5
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