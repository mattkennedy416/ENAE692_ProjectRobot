

time = 0:100;

theta7 = linspace(0, 2*pi, length(time));


time = transpose(time);
theta7 = transpose(theta7);

T = table(time, theta7);

makeRoboWorksAnimation(T, 'theta2Animation');









