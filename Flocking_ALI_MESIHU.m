%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS - 3A ASI -----------------
%--------------- Lara BRINON ARRANZ ------------------

% ----------------- STUDENTS FILE --------------------
%% Flocking for double integrator robots

% robot model
% pi = [xi;yi]
% vi = [vix;viy]
% double integrator
% pi_dot = vi
% vi_dot = ui

init;

% simulation parameters
%number of robots
N=5;

% sampling time
dt=1/(2*N); % DO NOT MODIFY

% iterations
T=200;


% Variables to be used in the simulation
% vector of all robot states P=[p1;p2;...;pN] 
% X=[x1;x2;...;xN] Y=[y1;y2;...;yN]
% We keep in memory the values at each iteration k
X=zeros(N,T);
Y=zeros(N,T);

% random robot initial states (k=1)
P0=20*randn(N,2);
X(:,1)=P0(:,1);
Y(:,1)=P0(:,2);

V0=5+5*randn(N,2);
Vx=V0(:,1);
Vy=V0(:,2);

%All-to-All Communication
A=ones(N)-eye(N);
D=(N-1)*eye(N);
L=D-A;

% vector of all control inputs U=[u1;u2;...;uN]
% Ux=[ux1;ux2;...;uxN] Uy=[uy1;uy2;...;uyN]
% we keep in memory the values of U at each iteration k
% initial conditions for control inputs (k=1)
Ux=zeros(N,T);
Uy=zeros(N,T);


% SIMULATION
for k=2:T
    clf();
    axis([-60,60,-60,60]);
    axis square; hold on

    % control parameters
    alpha=1;
    beta=1;
    gamma=15;

    % communication radius
    rho=20;
    L = zeros(N,N);
    for i=1:N    
        for j=1:N
            if i~=j
                vector = [X(i,k-1)-X(j,k-1) Y(i,k-1)-Y(j,k-1)];
                if norm(vector) < rho
                    L(i,j) = -1;
                else
                    L(i,j) = 0;
                end
            end
        end
    end
    for i=1:N
        L(i,i) = -sum(L(i,:));
    end

    for i=1:N
        % control law for robot i
        Ux(i,k-1)=0;
        Uy(i,k-1)=0;
        
        % alignment
        % write your code here
        Ux = -alpha*L*Vx(:,k-1);
        Uy = -alpha*L*Vy(:,k-1);
        
        % cohesion
        % write your code here
        Ux = Ux - beta*L*X(:,k-1);
        Uy = Uy - beta*L*Y(:,k-1);

        % separation
        % write your code here
        for j=1:N
            if i ~= j
                vector = [X(i,k-1)-X(j,k-1) Y(i,k-1)-Y(j,k-1)];
                seperation = vector/(norm(vector)^3);
                Ux(i) = Ux(i) + gamma*seperation(1);
                Uy(i) = Uy(i) + gamma*seperation(2);
            end
        end
        
        
        % update the state of robot i using its dynamics
        % modify this code
        Vx(i,k)=Vx(i,k-1) + Ux(i)*dt;
        Vy(i,k)=Vy(i,k-1) + Uy(i)*dt;
        X(i,k)=X(i,k-1) + Vx(i,k)*dt;
        Y(i,k)=Y(i,k-1) + Vy(i,k)*dt;

        % draw the robots as circles
        plot(X(i,k),Y(i,k),'oblack','LineWidth',3)
        % draw the robots' velocities as arrows
        quiver(X(i,k),Y(i,k),Vx(i,k),Vy(i,k),'LineWidth',2)
    
    end
    drawnow();
end

%% Figure
figure 
% plot the robots' initial states in red
plot(X(:,1),Y(:,1),'ored','LineWidth',3);
title('Flocking')
xlabel('x')
ylabel('y')
hold on
axis square;
for i=1:N
    quiver(X(i,1),Y(i,1),Vx(i,1),Vy(i,1),'r','LineWidth',1)
end

% plot the robots' trajectories in black
for i=1:N
    plot(X(i,:),Y(i,:),'k','LineWidth',1);
end

% plot the robots' final states in blue
plot(X(:,T),Y(:,T),'ob','LineWidth',3);
for i=1:N
    quiver(X(i,T),Y(i,T),Vx(i,T),Vy(i,T),'b','LineWidth',1)
end
title('Movement of Robots in Simulation');
xlabel('X Position (m)')
ylabel('Y Position (m)')
hold off

figure
% plot the robots' velocities over time
subplot(2,1,1);
% TO DO
hold on
for i = 1:N
    plot(Vx(i,:))
end
title('Velocities of Robots over Time in X and Y axis')
xlabel('Time (s)')
ylabel('X Velocity (m/s)')
subplot(2,1,2); 
% TO DO
hold on
for j = 1:N
    plot(Vy(j,:))
end
xlabel('Time (s)')
ylabel('Y Velocity (m/s)')
hold off
   
figure
% plot the robots' positions over time
subplot(2,1,1);
% TO DO
hold on
for i = 1:N
    plot(X(i,:))
end
title('Positions of Robots over Time in X and Y axis')
xlabel('Time (s)')
ylabel('X Position (m)')
subplot(2,1,2);
% TO DO
hold on
for i = 1:N
    plot(Y(i,:))
end
xlabel('Time (s)')
ylabel('Y Position (m)')