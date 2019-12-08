

function makeRoboWorksAnimation(inputTable, outputName)

isTableCol = @(t, thisCol) ismember(thisCol, t.Properties.VariableNames);

% take in some table of whatever parameters are varying, assume the rest
% are the default values


% set the default values - note that these need to be in radians since we
% convert to degrees below
d.ship_roll = 0;
d.ship_pitch = 0;
d.ship_yaw = 0;
d.d1 = 50;
d.theta2 = 0;
d.theta3 = pi/2;
d.d4 = 50;
d.theta5 = 0;
d.theta6 = 0;
d.theta7 = 0;
d.catcher = 0;
d.x = 400;
d.y = -750;
d.z = 0;
d.drone_roll = 0;
d.drone_pitch = 0;
d.drone_yaw = 0;


time = inputTable.time; % this is required, use it as the length

fields = fieldnames(d);
for n = 1:length(fields)
    % need to check what data is available and what we should use the
    % default value. terrible way to do this, but hey it's matlab
    field = string(fields(n));
    
    if isTableCol(inputTable, field)
        eval(strcat(field,' = inputTable.',field,';'));
    else
        eval(strcat(field,' = d.',field,'*ones(length(time),1);'));
    end
    
end

% and these are all going to be in radians, roboworks wants degrees
ship_roll = rad2deg(ship_roll);
ship_pitch = rad2deg(ship_pitch);
ship_yaw = rad2deg(ship_yaw);
theta2 = rad2deg(theta2);
theta3 = rad2deg(theta3);
theta5 = rad2deg(theta5);
theta6 = rad2deg(theta6);
theta7 = rad2deg(theta7);
drone_roll = rad2deg(drone_roll);
drone_pitch = rad2deg(drone_pitch);
drone_yaw = rad2deg(drone_yaw);

outputTable = table(ship_roll, ship_pitch, ship_yaw, d1, theta2, theta3, d4, theta5, theta6, theta7, catcher, x, y, z, drone_roll, drone_pitch, drone_yaw);

outputFilename = strcat('../animations/', outputName, '.dat');
writetable(outputTable,outputFilename,'Delimiter','\t');  





end

