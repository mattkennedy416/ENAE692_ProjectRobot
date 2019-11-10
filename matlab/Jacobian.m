
% calculate the Translational Jacobian (Base Frame)
% this analytical solution was calculated by FwdKin_myRobot.nb

function [J, J_tran] = Jacobian(dh, pT)

% where dh is the dh table and pT is the 3x1 vector of tool tip position
% relative to origin 7

% sometimes we only want the translational Jacobian so just return both

J_tran = J_translation(dh, pT);
J_rot = J_rotation(dh, pT);

J = cat(1, J_tran, J_rot);

end


function J = J_rotation(dh, pT)

% where dh is the dh table and pT is the 3x1 vector of tool tip position
% relative to origin 7

th = dh.theta_i;

s = sin(th);
c = cos(th);
s35 = sin(th(3)+th(5));
c35 = cos(th(3)+th(5));

J = zeros(3,7);

J(1,1) = 0;
J(1,2) = 0;
J(1,3) = s(2);
J(1,4) = 0;
J(1,5) = s(2);
J(1,6) = c(2)*s35;
J(1,7) = -c(7)*s(2) + c(2)*c35*s(7);
J(2,1) = 0;
J(2,2) = 0;
J(2,3) = -c(2);
J(2,4) = 0;
J(2,5) = -c(2);
J(2,6) = s(2)*s35;
J(2,7) = c(2)*c(7) + c35*s(2)*s(7);
J(3,1) = 0;
J(3,2) = 1;
J(3,3) = 0;
J(3,4) = 0;
J(3,5) = 0;
J(3,6) = -c35;
J(3,7) = s35 * s(7);

end




function J = J_translation(dh, pT)

% where dh is the dh table and pT is the 3x1 vector of tool tip position
% relative to origin 7

a = dh.a_i1;
al = dh.alpha_i1;
d = dh.d_i;
th = dh.theta_i;

s = sin(th);
c = cos(th);
s35 = sin(th(3)+th(5));
c35 = cos(th(3)+th(5));

J = zeros(3,7);

J(1,1) = 0;
J(1,2) = -s(2)*a(2)-s(2)*s(3)*d(4)-cos(th(3)+th(5))*(c(7)^2)*s(2)*pT(1)+c(2)*c(7)*s(7)*pT(1)-s(2)*sin(th(3)+th(5))*s(7)*pT(1) ...
    - c(7)*s(2)*sin(th(3)+th(5))*pT(2) + cos(th(3)+th(5))*c(7)*s(2)*s(7)*pT(2) - c(2)*s(7)^2 * pT(2) - c(2)*c(7)*pT(3)-cos(th(3)+th(5))*s(2)*s(7)*pT(3);

J(1,3) = c(2)*( c(3)*d(4) + ( -c(7)^2*s35 + c35*s(7) )*pT(1) + c35*c(7)*pT(2) + c(7)*s35*s(7)*pT(2) - s35*s(7)*pT(3) );

J(1,4) = c(2)*s(3);

J(1,5) = c(2)* ( ( -c(7)^2*s35 + c35*s(7) )*pT(1) + c(7)*(c35 + s35*s(7))*pT(2) - s35*s(7)*pT(3) );

J(1,6) = ( c(7)^2*s(2) + c(2)*c(7)*s35-s(2)*s(7)^2 - c(2)*c35 * sin(2*th(7)) ) * pT(1) ...
    - ( c(2)* ( c35*cos(2*th(7)) + s35*s(7) ) + s(2)*sin(2*th(7)) ) * pT(2) ...
    + ( c(2)*c35 *c(7) + s(2)*s(7) ) * pT(3);

J(1,7) = ( c(7)^2 * s(2) + c(2)*c(7)*s35 - s(2)*s(7)^2 - c(2)*c35 * sin(2*th(7)) )*pT(1) ...
    - ( c(2)* ( c35*cos(2*th(7)) + s35 * s(7) ) + s(2)*sin(2*th(7)) )*pT(2) ...
    + ( c(2)*c35*c(7) + s(2)*s(7) ) * pT(3); 

J(2,1) = 0;
J(2,2) = c(2)*a(2) + c(2)*s(3)*d(4)+c(2)*c35*c(7)^2 * pT(1) ...
    + c(7)*s(2)*s(7) * pT(1) ...
    + c(2)*s35 * s(7) * pT(1) ...
    + c(2)*c(7)*s35*pT(2) ...
    - s(2)*s(7)^2 * pT(2) ...
    - 0.5*c(2)*c35*sin(2*th(7)) * pT(2) ...
    - c(7)*s(2) * pT(3) ...
    + c(2)*c35*s(7)*pT(3);

J(2,3) = 0.5*s(2) * ( 2*c(3)*d(4) + (-2*c(7)^2 * s35 + 2*c35*s(7) ) * pT(1) ...
    + 2*c35*c(7)*pT(2) ...
    + s35*sin(2*th(7))*pT(2) ...
    - 2*s35 * s(7) * pT(3) );

J(2,4) = s(2)*s(3);

J(2,5) = 0.5*s(2) * ( ( -c(7)^2 * s35 + 2*c35*s(7) ) * pT(1) ...
    + (2*c35 * c(7) + s35*sin(2*th(7)))*pT(2) ...
    -2*s35 * s(7) * pT(3) );

J(2,6) = -(c(2)*cos(2*th(7)) + s(2)*(-c(7)*s35 + c35*sin(2*th(7)) ) ) * pT(1) ...
    - ( c35 * cos(2*th(7))*s(2) + s(2)*s35*s(7) - c(2)*sin(2*th(7)) ) * pT(2) ...
    + ( c35*c(7)*s(2) - c(2)*s(7) ) * pT(3);

J(2,7) = -( c(2)*cos(2*th(7)) + s(2) * ( -c(7)*s35 + c35*sin(2*th(7)) ) ) * pT(1) ...
    - ( c35 * cos(2*th(7)) * s(2) + s(2)*s35 * s(7) - c(2)*sin(2*th(7)) )*pT(2) ...
    + ( c35 * c(7)*s(2) - c(2)*s(7) ) * pT(3);

J(3,1) = 1;
J(3,2) = 0;

J(3,3) = s(3)*d(4) + ( c35*c(7)^2 + s35*s(7) ) * pT(1) ...
    + c(7)*s35*pT(2) ...
    - c35 * c(7) * s(7) * pT(2) ...
    + c35 * s(7) * pT(3);

J(3,4) = -c(3);

J(3,5) = ( c35 * c(7)^2 + s35*s(7) ) * pT(1) ...
    + c(7)*(s35 - c35*s(7)) * pT(2) ...
    + c35 * s(7) * pT(3);

J(3,6) = -(c35 * c(7) * s35 * sin(2*th(7)) )*pT(1) ...
    + (-c(7)^2 * s35 + s(7)*( c35 + s35 * s(7) ) ) * pT(2) ...
    + c(7)*s35 * pT(3);

J(3,7) = -(c35 * c(7) * s35 * sin(2*th(7)) )*pT(1) ...
    + (-c(7)^2 * s35 + s(7)*( c35 + s35 * s(7) ) ) * pT(2) ...
    + c(7)*s35 * pT(3);


end












