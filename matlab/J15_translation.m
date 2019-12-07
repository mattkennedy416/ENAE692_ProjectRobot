

function J = J15_translation(dh)

% transformation jacobian to origin 5 (end of arm)

a = dh.a_i1;
al = dh.alpha_i1;
d = dh.d_i;
th = dh.theta_i;

s = sin(th);
c = cos(th);
s35 = sin(th(3)+th(5));
c35 = cos(th(3)+th(5));

J = zeros(3,5);


J(1,1) = 0;
J(1,2) = -s(2)*(a(2) + s(3)*d(4));
J(1,3) = c(2)*c(3)*d(4);
J(1,4) = c(2)*s(3);
J(1,5) = 0;

J(2,1) = 0;
J(2,2) = c(2)*( a(2) + s(3)*d(4));
J(2,3) = c(3)*s(2)*d(4);
J(2,4) = s(2)*s(3);
J(2,5) = 0;

J(3,1) = 1;
J(3,2) = 0;
J(3,3) = s(3)*d(4);
J(3,4) = -c(3);
J(3,5) = 0;

end

