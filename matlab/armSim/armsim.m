%    THIS PROGRAM SIMULATES THE TWO-LINK ARM USING A
%    DISCRETE-TIME CONTROLLER

Tint = 0.005 ;						% integration period
Tcon = 0.020 ;						% control period
Tplot = 0.01 ;						% plotting period
Ncon = round(Tcon/Tint) ;
Nplot = round(Tplot/Tint) ;
Ttotal = 5.0-Tcon ;

% ACTUAL PARAMETERS
L = [0.5; 0.5];  % link length (m)
m = [1; 1] ;  % link mass (kg)
mu = [0; 0] ;  % viscous friction coefficient (N-m/rad/sec)
g = 9.8 ;  % gravitational acceleration (m/2^2)

% MODEL PARAMETERS
L_mod = [0.5; 0.5] ;
m_mod = [1; 1] ;
mu_mod = [0; 0] ;

% SET THE CONTROL GAINS (Exer 10.3)
Kp = [16 0; 0 16] ;
Kv = [8 0; 0 8] ;

%  INITIALIZE TIME, STATE, AND CONTROL

t = 0.0 ;
x = [0 0 0.0 0.0] ;
u = [0.0 0.0] ;
q = x(1:2);
qd = x(3:4);
q_des(1) = pi/2;
q_des(2) = 0;
qd_des(1) = 0;
qd_des(2) = 0;
qdd_des(1) = 0;
qdd_des(2) = 0;

	  	
%  INITIALIZE COUNTERS FOR PLOTTING, INTEGRATION, AND CONTROL

i = 1 ;								% plot counter
cycle = 0 ;							% integration cycles
rkcounter = 1 ;						% integration counter

%  RUN INTEGRATION

while t < Ttotal

	disp(t) ;
	
	% Update the Trajectory
	
% 	[q_des,qd_des,qdd_des] = trajectory(t) ;

	% Sample the State
	
	xsampled = x ;
	
    % UPDATE THE CONTROL:  Choose one of these three controllers and comment out the others

    % (a) No Control
%     u = [0 0] ;
    % (b) DJC Control
	 u = control_djc( t, xsampled, q_des, qd_des, qdd_des, L_mod, m_mod, mu_mod, g, Kp, Kv) ;
    % (c) IJC Control
   u = control_ijc( t, xsampled, q_des, qd_des, qdd_des, L_mod, m_mod, mu_mod, g, Kp, Kv) ;

	% Integrate over Ncon*Tint seconds
	
	while rkcounter <= Ncon
		
	% Update the plotted variables (if it's time)
	
		if ( round(cycle/Nplot)*Nplot == cycle )
			time(i) = t ;
			state(i,:) = x ;
			control(i,:) = u ;
			error(i,:) = q_des - xsampled(1:2) ;
            q = x(1:2) ;
            position(i,:) = fwdkin(q,L) ;
			i = i + 1 ;
		end
	
	% Call Runge-Kutta Integrator
		
		[x,u] = RK4_discrete(Tint,t,q_des,qd_des,qdd_des,u,x,L,m,mu,g) ;
		t = t + Tint ;
		rkcounter = rkcounter + 1 ;
		
	% Update time counter
	
		cycle = cycle + 1 ;
		
	end
	
	rkcounter = 1 ;
	
end


% PLOT DATA

plot(time,state(:,1:2)) ;
xlabel('time (sec)') ;
ylabel('q (rad)') ;
title('Joint Position') ;
pause ;
plot(time,error) ;
xlabel('time (sec)') ;
ylabel('q_error (rad)') ;
title('Joint Position Error') ;
pause ;
plot(time,control) ;
xlabel('time (sec)') ;
ylabel('T (N-m)') ;
title('Joint Torque') ;
pause ;
plot(position(:,1),position(:,2)) ;
xlabel('x (m)') ;
ylabel('y (m)') ;
title('Tool Position') ;
pause;
                                    
% animation
%figure(4)
%axis([-1,1,-1,1])
%h=animatedline('MaximumNumPoints',3)
%h.Marker='o'
%for k=1:500
%    addpoints(h,0,0)
%    addpoints(h,position(k,3),position(k,4));
%    addpoints(h,position(k,1),position(k,2));
%    drawnow
%end
figure(4)
tic
for k=1:500
    tic
    x0=[0,position(k,3),position(k,1)];
    x1=[0,position(k,4),position(k,2)];
    plot(x0,x1,'o-')
    axis([-1,1,-1,1])
    drawnow
end
