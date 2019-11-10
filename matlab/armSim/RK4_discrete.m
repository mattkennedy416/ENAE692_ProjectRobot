%  DISCRETE-TIME 4TH ORDER RUNGE-KUTTA INTEGRATORfunction [x,u] = RK4_discrete( dt, t, q_des, qd_des, qdd_des, u, x, L, m, mu, g )k0 = xdot(t,x,u,L,m,mu,g) ;		ti = t + dt/2.0 ;xi = x + k0*dt/2.0 ;k1 = xdot(ti,xi,u,L,m,mu,g) ;		ti = t + dt/2.0 ;xi = x + k1*dt/2.0 ;k2 = xdot(ti,xi,u,L,m,mu,g) ;		ti = t + dt ;xi = x + k2*dt ;k3 = xdot(ti,xi,u,L,m,mu,g) ;		x = x + (k0+2.0*k1+2.0*k2+k3)*dt/6.0 ;