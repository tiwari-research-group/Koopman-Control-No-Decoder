% Koopman with Control for Pendulum Example

clear
clc

% discrete time domain Koopman operator 
K_org = [ 1.0000e+00,  9.9944e-03, -1.0609e-03,  2.9259e-03;
         2.2888e-02,  9.9959e-01, -3.4480e-01,  9.4425e-01;
        -1.6365e-04,  2.5746e-04,  1.0015e+00, -4.2073e-03;
         1.4503e-04, -2.7955e-04, -1.3345e-03,  1.0036e+00];

A = (K_org-eye(4))/0.01; % transforming from discrete to continuous time domain

% discrete time domain Koopman B matrix
B = [1.4323e-04;
     1.0000e-02;
     1.4402e-06;
     9.8776e-07];

B = B/0.01; % transforming from discrete to continuous time domain

% checking controllability of system
Co = ctrb(A,B);
unco = length(A) - rank(Co);
if unco == 0 
    disp('The System is Controllable')
else 
    disp('The System is Uncontrollable')
end

% Setting up LQR parameters and weight matrices
R = 1; 

Q = 1*eye(4); 

Q(1,1) = 1;

Q(2,2) = 1;

Q(3,3) = 0; 

Q(4,4)  = 0; 

[K,S,P] = lqr(A,B,Q,R); % Solving LQR control problem


%% Intergrating LQR solution for 20 seconds
tspan = 0:0.01:20;

x_in = [1.4021;
       -1.0130;
       -0.4645;
       -0.2237];

[t,x_lqr] = ode45(@(t,x) (A-B*K)*x, tspan,x_in); 

%% Plotting

figure 
subplot(2,1,1)
plot(t,x_lqr(:,1), 'LineWidth', 2)
title("LQR Control Response for $$\theta$$", 'interpreter', 'latex', 'fontsize', 14)
ylabel("$$\theta$$ [rad]", 'interpreter', 'latex')
xlabel("Time [sec]", 'interpreter', 'latex')
% xticklabels({'0','5','10','15'})
ax = gca;
ax.FontName = 'Cambria Math';
ax.FontSize = 14;
% ylim([-5 20])

subplot(2,1,2)
plot(t,x_lqr(:,2), 'LineWidth', 2)
title("LQR Control Response for $$\dot{\theta}$$", 'interpreter', 'latex', 'fontsize', 14)
ylabel("$$\dot{\theta}$$ [rad/s]", 'interpreter', 'latex')
xlabel("Time [sec]", 'interpreter', 'latex')
% xticklabels({'0','5','10','15'})
ax = gca;
ax.FontName = 'Cambria Math';
ax.FontSize = 14;
% ylim([-8 15])

% writing LQR state history to .csv file
writematrix(x_lqr,'LQR_data.csv') 
%% Propgation with A_org (Discrete Dynamics Propogation) 

x_org = [];

x_in = [0.3825;
       -1.1557;
        0.4990;
       -0.0063];

t_org = 0:0.01:100;


for i = 0:0.01:100
    x_org = [x_org, x_in]; 
    x_next = (K_org-B*0.01*K)*x_in; 
    x_in = x_next;
end 

figure 
plot(t_org,x_org(1,:))
hold on 
plot(t_org,x_org(2,:))
