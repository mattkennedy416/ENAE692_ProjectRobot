

% he wants an animation of each of the joints working individually

% we also need to define the joint limits here

jointLimits = [50, 75;
    -pi, pi;
    pi/4, 3*pi/4;
    50, 75;
    -pi/4, pi/4;
    -pi/8, pi/8;
    -pi/4, 5*pi/4];


time = 0:100;
time = transpose(time);


d1 = transpose(linspace(jointLimits(1,1), jointLimits(1,2), length(time)));
theta2 = transpose(linspace(jointLimits(2,1), jointLimits(2,2), length(time)));

theta3 = transpose(linspace(jointLimits(3,1), jointLimits(3,2), length(time)));
theta5 = pi/2 - theta3;

d4 = transpose(linspace(jointLimits(4,1), jointLimits(4,2), length(time)));

theta6 = transpose(linspace(jointLimits(6,1), jointLimits(6,2), length(time)));
theta7 = transpose(linspace(jointLimits(7,1), jointLimits(7,2), length(time)));



makeRoboWorksAnimation(table(time, d1), 'd1Animation');
makeRoboWorksAnimation(table(time, theta2), 'theta2Animation');

makeRoboWorksAnimation(table(time, theta3, theta5), 'theta3-5Animation');

makeRoboWorksAnimation(table(time, d4), 'd4Animation');
makeRoboWorksAnimation(table(time, theta6), 'theta6Animation');
makeRoboWorksAnimation(table(time, theta7), 'theta7Animation');



