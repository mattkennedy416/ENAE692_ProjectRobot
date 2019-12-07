

function DH = dhParameters(state)

% fixed parameters
a2 = state.a2;
a5 = state.a5;
d6 = state.d6;

% variable parameters
d1 = state.d1;
theta2 = state.theta2;
theta3 = state.theta3;
d4 = state.d4;
theta5 = state.theta5;
theta6 = state.theta6;
theta7 = state.theta7;



alpha_i1 = [0; 0; pi/2; pi/2; -pi/2; pi/2; pi/2];
a_i1 = [0; 0; a2; 0; 0; a5; 0];
d_i = [d1; 0; 0; d4; 0; d6; 0];
theta_i = [0; theta2; theta3; 0; theta5; theta6; theta7];


DH = table(alpha_i1, a_i1, d_i, theta_i);

end
