%  COMPUTES STATE DERIVATIVE FOR PLANAR TWO-LINK MANIPULATORfunction xd = xdot ( t, x, u, L, m, mu, g )q(1:2) = x(1:2) ;qdot(1:2) = x(3:4) ;c1 = cos(q(1)) ;c2 = cos(q(2)) ;s2 = sin(q(2)) ;c12 = cos(q(1)+q(2)) ;	M(1,1) = (m(1)+m(2))*L(1)^2 + m(2)*L(2)^2 + 2.0*m(2)*L(1)*L(2)*c2 ;M(1,2) = m(2)*L(2)^2 + m(2)*L(1)*L(2)*c2 ;M(2,1) = M(1,2) ;M(2,2) = m(2)*L(2)^2 ;	V(1) = -m(2)*L(1)*L(2)*s2*(2.0*qdot(1)*qdot(2)+qdot(2)^2) ;V(2) = m(2)*L(1)*L(2)*s2*qdot(1)^2 ;	G(1) = (m(1)+m(2))*g*L(1)*c1 + m(2)*g*L(2)*c12 ;G(2) = m(2)*g*L(2)*c12 ;f(1) = mu(1) *qdot(1) ;f(2) = mu(2) *qdot(2) ;det = M(1,1)*M(2,2)-M(1,2)*M(2,1) ;Minv(1,1) = M(2,2)/det ;Minv(1,2) = - M(2,1)/det ;Minv(2,1) = - M(1,2)/det ;Minv(2,2) = M(1,1)/det ;qdd = Minv * ( u' - V' - G' - f') ;xd(1:2) = x(3:4) ;xd(3:4) = qdd(1:2) ;