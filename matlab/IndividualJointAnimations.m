

% he wants an animation of each of the joints working individually

% we also need to define the joint limits here

jointLimits = [50, 75;
    -pi, pi;
    pi/4, 3*pi/4;
    50, 75;
    -pi/4, pi/4;
    -pi/8, pi/8;
    -pi, pi];

defaults = [50; 0; pi/2; 50; 0; 0; 0];



timeBase = 0:100;
timeBase = transpose(timeBase);


d1 = transpose(linspace(jointLimits(1,1), jointLimits(1,2), length(timeBase)));
timePerc = 1 - (defaults(1) - jointLimits(1,1))/(jointLimits(1,2)-jointLimits(1,1));
timeReturn = transpose(timeBase(end) : timeBase(end) + round(length(timeBase)*timePerc));
d1Return = transpose(linspace(jointLimits(1,2), defaults(1), length(timeReturn)));
time = [timeBase; timeReturn];
d1 = [d1; d1Return];
makeRoboWorksAnimation(table(time, d1), 'd1Animation');

n=2;
theta2 = transpose(linspace(jointLimits(n,1), jointLimits(n,2), length(timeBase)));

timePerc = 1 - (defaults(n) - jointLimits(n,1))/(jointLimits(n,2)-jointLimits(n,1));
timeToMin = transpose(0:round(length(timeBase)*timePerc));
valuesToMin = transpose(linspace(defaults(n), jointLimits(n,1), length(timeToMin)));

timeReturn = transpose(timeBase(end) : timeBase(end) + round(length(timeBase)*timePerc));
returnValues = transpose(linspace(jointLimits(n,2), defaults(n), length(timeReturn)));
time = [timeToMin; timeToMin(end)+timeBase; timeReturn];
theta2 = [valuesToMin; theta2; returnValues];
makeRoboWorksAnimation(table(time, theta2), 'theta2Animation');


n=3;
theta3 = transpose(linspace(jointLimits(n,1), jointLimits(n,2), length(timeBase)));
timePerc = 1 - (defaults(n) - jointLimits(n,1))/(jointLimits(n,2)-jointLimits(n,1));
timeToMin = transpose(0:round(length(timeBase)*timePerc));
valuesToMin = transpose(linspace(defaults(n), jointLimits(n,1), length(timeToMin)));
timeReturn = transpose(timeBase(end) : timeBase(end) + round(length(timeBase)*timePerc));
returnValues = transpose(linspace(jointLimits(n,2), defaults(n), length(timeReturn)));
time = [timeToMin; timeToMin(end)+timeBase; timeReturn];
theta3 = [valuesToMin; theta3; returnValues];
theta5 = pi/2 - theta3;
makeRoboWorksAnimation(table(time, theta3, theta5), 'theta3-5Animation');


n=4;
d4 = transpose(linspace(jointLimits(n,1), jointLimits(n,2), length(timeBase)));
timePerc = 1 - (defaults(n) - jointLimits(n,1))/(jointLimits(n,2)-jointLimits(n,1));
% timeToMin = transpose(0:round(length(timeBase)*timePerc));
% valuesToMin = transpose(linspace(defaults(n), jointLimits(n,1), length(timeToMin)));
timeReturn = transpose(timeBase(end) : timeBase(end) + round(length(timeBase)*timePerc));
returnValues = transpose(linspace(jointLimits(n,2), defaults(n), length(timeReturn)));
time = [timeToMin(end)+timeBase; timeReturn];
d4 = [d4; returnValues];
makeRoboWorksAnimation(table(time, d4), 'd4Animation');


n=6;
theta6 = transpose(linspace(jointLimits(n,1), jointLimits(n,2), length(timeBase)));
timePerc = 1 - (defaults(n) - jointLimits(n,1))/(jointLimits(n,2)-jointLimits(n,1));
timeToMin = transpose(0:round(length(timeBase)*timePerc));
valuesToMin = transpose(linspace(defaults(n), jointLimits(n,1), length(timeToMin)));
timeReturn = transpose(timeBase(end) : timeBase(end) + round(length(timeBase)*timePerc));
returnValues = transpose(linspace(jointLimits(n,2), defaults(n), length(timeReturn)));
time = [timeToMin; timeToMin(end)+timeBase; timeReturn];
theta6 = [valuesToMin; theta6; returnValues];
makeRoboWorksAnimation(table(time, theta6), 'theta6Animation');


n=7;
theta7 = transpose(linspace(jointLimits(n,1), jointLimits(n,2), length(timeBase)));
timePerc = 1 - (defaults(n) - jointLimits(n,1))/(jointLimits(n,2)-jointLimits(n,1));
timeToMin = transpose(0:round(length(timeBase)*timePerc));
valuesToMin = transpose(linspace(defaults(n), jointLimits(n,1), length(timeToMin)));
timeReturn = transpose(timeBase(end) : timeBase(end) + round(length(timeBase)*timePerc));
returnValues = transpose(linspace(jointLimits(n,2), defaults(n), length(timeReturn)));
time = [timeToMin; timeToMin(end)+timeBase; timeReturn];
theta7 = [valuesToMin; theta7; returnValues];
makeRoboWorksAnimation(table(time, theta7), 'theta7Animation');









