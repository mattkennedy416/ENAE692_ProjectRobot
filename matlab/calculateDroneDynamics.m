

function droneState = calculateDroneDynamics(targetPosition, robotBasePosition, dt)

% for now, just make the drone come straight in with no turns

droneHookPosition = [0; -5; 10]; % relative position of the hook on the drone (that needs to hit the target)

targetPosition(2) = -targetPosition(2); % since the y axis is flipped
absTargetPosition = robotBasePosition + targetPosition - droneHookPosition; % target in abs coordinates

velMag = 10;
droneRelInitialPosition = [0; -300; 50];
velocity = -velMag * droneRelInitialPosition/norm(droneRelInitialPosition);


droneInitialPosition = absTargetPosition + droneRelInitialPosition;


pitchAngle = deg2rad(5); % just keep this constant?
rollAngle = 0; % just keep this constant?
yawAngle = 0;

x = droneInitialPosition(1);
y = droneInitialPosition(2);
z = droneInitialPosition(3);
drone_yaw = yawAngle;
drone_pitch = pitchAngle;
drone_roll = rollAngle;

time = 0;

maxIter = 500;
for iter = 2:maxIter
   
    time(iter,1) = (iter-1) * dt;
    x(iter,1) = x(iter-1) + velocity(1) * dt; 
    y(iter,1) = y(iter-1) + velocity(2) * dt;
    z(iter,1) = z(iter-1) + velocity(3) * dt;
    
    dx = x(iter) - x(iter-1);
    dy = y(iter) - y(iter-1);
    drone_yaw(iter,1) = atan(dx / dy);
    
    drone_pitch(iter,1) = pitchAngle;
    drone_roll(iter,1) = rollAngle;
    
    if y(iter) > absTargetPosition(2)
        break
    end  
    
end

if y(iter) < absTargetPosition(2)
    fprintf("DID NOT CONVERGE!\n");
end

droneState = table(time, x,y,z, drone_pitch, drone_yaw, drone_roll);

end

