
function [combinedState, catcherState] = calculateCatcherDynamics(droneState, dt)

% so droneState should have stopped when it hit the target
% and hopefully armState is already at the target ...

% though this is really going to be independent of either the armState or
% droneState, as long as we actually hit the target

% but I guess we'll start from the last droneState and assume that we captured

% the catcher mechanism starts at 0 and goes to -55


dx = droneState.x(end) - droneState.x(end-1);
dy = droneState.y(end) - droneState.y(end-1);
dz = droneState.z(end) - droneState.z(end-1);

% vel_initial = sqrt(dx^2 + dy^2 + dz^2);
vel_initial = [dx, dy, dz];

maxDisplacement = 50; % hard limit at 55, but we may go slightly over
maxHookLength = 4;

% and we'll just have a linear deceleration

vel_final = vel_initial / 2;

totalTime = maxDisplacement / ((norm(vel_final) + norm(vel_initial))/2);

acc = (vel_final.^2 - vel_initial.^2)/(2*maxDisplacement);

dronePos = [droneState.x(end), droneState.y(end), droneState.z(end)];
vel = vel_initial;
pitch = droneState.drone_pitch(end);
for iter = 2:500    
    dronePos(iter,:) = dronePos(iter-1,:) + vel*dt + 0.5*acc*dt^2;
    vel = vel + acc*dt;
    
    catcherPosition(iter,1:2) = dronePos(iter,1:2) - dronePos(1,1:2);
    
    if abs(dronePos(iter,3) - dronePos(1,3)) > maxHookLength
        acc(3) = 0;
        vel(3) = 0;
    end
    if acc(3) == 0 && pitch(iter-1) < deg2rad(45)
        pitch(iter,1) = pitch(iter-1) + deg2rad(3);
    else
        pitch(iter,1) = pitch(iter-1);
        dronePos(iter,2) = dronePos(iter,2) + 0.5;
        dronePos(iter,3) = dronePos(iter,3) + 0.1;
    end
    
    
    if norm(vel) < norm(vel_final)
        break
    end
    
end

% I guess lets just append this to the drone state?
time = dt : dt : size(dronePos,1)* dt;
time = transpose(time) + droneState.time(end);

x = dronePos(:,1);
y = dronePos(:,2);
z = dronePos(:,3);

% drone_pitch = droneState.drone_pitch(end) * ones(length(x),1);
drone_pitch = pitch;
drone_yaw = droneState.drone_yaw(end) * ones(length(x),1);
drone_roll = droneState.drone_roll(end) * ones(length(x),1);

newTable = table(time, x,y,z, drone_pitch, drone_yaw, drone_roll);

combinedState = [droneState; newTable];



catcher = -sqrt(catcherPosition(:,1).^2 + catcherPosition(:,2).^2);
catcherState = zeros(size(combinedState,1), 1);
catcherState(end-length(catcher)+1:end) = catcher; % add this in at the end



end