%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS - 3A ASI -----------------
%--------------- Lara BRINON ARRANZ ------------------

% ---------------- STUDENTS FILE ---------------------
%% Consensus simple integrator robots

% position of robot i (2D)
% pi = [xi;yi]
% robot model: simple integrator
% pi_dot = ui = [uxi;uyi]

% Select the question to be simulated
question = 'Question2';
switch question
    case 'Question2' % Question 2: all-to-all communication
        % number of robots
        N=5; % N should be =< 100
        % Adjacency matrix
        A=ones(N,N)-eye(N);
        % Degree matrix
        D=(N-1)*eye(N);
        % Laplacian matrix
        L=D-A;
        
    case 'Question3' % Question 3: different communication graphs
        % number of robots
        N=4;
        % Select the graph of the network to be simulated       
        graph_type = 'G1';
        switch graph_type
            case 'G1'
                A=[0 1 1 1;1 0 1 1;1 1 0 1;1 1 1 0];
                D=diag([3,3,3,3]);
                L=D-A;
            case 'G2'
                A=[0 1 0 1;1 0 1 1;0 1 0 1;1 1 1 0];
                D=[2 0 0 0;0 3 0 0;0 0 2 0;0 0 0 3];
                L=D-A;
            case 'G3'
                A=[0 1 0 1;1 0 1 0;0 1 0 1;1 0 1 0];
                D=diag([2 2 2 2]);
                L=D-A;
            case 'G4'
                A=[0 1 0 0;1 0 1 0;0 1 0 1;0 0 1 0];
                D=[1 0 0 0; 0 1 0 0 ; 0 0 1 0;0 0 0 2];
                L=D-A;
                % direced graphs 
            case 'G5'
                A=[0 1 0 0;0 0 0 1; 0 1 0 0;1 0 1 0];
                D=[1 0 0 0;0 2 0 0;0 0 1 0;0 0 0 1];
                L=D-A;
             case 'G6'
                A=[0 1 0 0;0 0 0 0;0 1 0 0;0 0 1 0];
                D=[1 0 0 0;0 0 0 0 ;0 0 1 0;0 0 0 1];
                L=D-A;
             case 'G7'
                A=[1 0 0 0 ;0 0 0 0;0 1 0 0;0 0 0 0];
                D=[1 0 0 0;0 0 0 0;0 0 1 0;0 0 0 0];
                L=D-A;
        end
end

% simulation parameters
switch question
    case 'Question2' % Question 2: all-to-all communication
        % sampling time
        dt=1/(2*N); % max(N)=100
        % iterations
        T=200;
    case 'Question3' % Question 3: different communication graphs
        % sampling time
        dt=1/(2*N);
        % iterations
        T=100;
end

% Variables to be used in the simulation
% vector of all robot states P=[p1;p2;...;pN] 
% X=[x1;x2;...;xN] Y=[y1;y2;...;yN]
% We keep in memory the values at each iteration k
% X=zeros(N,T);
% Y=zeros(N,T);
X=zeros(N,1);
Y=zeros(N,1);
% vector of all control inputs U=[u1;u2;...;uN]
% We keep in memory the values at each iteration k
Ux=zeros(N,1);
Uy=zeros(N,1);

% initial conditions (k=1)
switch question
    case 'Question2' % Question 2: all-to-all communication
        % random initial conditions for Question 2
        P0=25*randn(N,2);
        X(:,1)=P0(:,1); 
        Y(:,1)=P0(:,2);
    case 'Question3' % Question 3: different communication graphs
        % initial conditions for Question 3
        X(:,1)=[42;-5;-54;-21];
        Y(:,1)=[34;-26;14;3];
end


% SIMULATION
for k=2:T
    clf();
    axis([-60,60,-60,60]);
    axis square; hold on

    % control parameters
    alpha=1; % consensus 
    beta=100;  % repulsive potential
    
        Ux =-L*X(:,k-1);
        Uy =-L*Y(:,k-1);
    for i=1:N
        % control law for robot i
        
        sum_x = 0 ;
        sum_y = 0 ;
        for j=1:N
            if i~=j
                norme = [X(i,k-1)-X(j,k-1) ; Y(i,k-1)-Y(j,k-1)]/norm([X(i,k-1)-X(j,k-1) ; Y(i,k-1)-Y(j,k-1)])^3 ;
                sum_x = sum_x - alpha*A(i,j)*(X(i,k-1)-X(j,k-1))  + beta*norme(1) ; %repulsive field : + beta*(X(i,k-1)-X(j,k-1)) / norme^3 ;
                sum_y = sum_y - alpha*A(i,j)*(Y(i,k-1)-Y(j,k-1)) + beta*norme(2) ; %repulsive field : + beta*(Y(i,k-1)-Y(j,k-1)) / norme^3 ;
            end
        end
        Ux(i)=sum_x;
        Uy(i)=sum_y;
        switch question
            case 'Question2' % Question 2: all-to-all communication
                % Question 2.3 
                % code your consensus algorithm here 
                    X(i,k)=X(i,k-1)+Ux(i)*dt;
                    Y(i,k)=Y(i,k-1)+Uy(i)*dt;

                % Question 2.4
                % consensus with Laplacian

            case 'Question3' % Question 3: consensus with Laplacian
                % code your consensus algorithm here
               X(i,k)=X(i,k-1)+Ux(i)*dt;
               Y(i,k)=Y(i,k-1)+Uy(i)*dt;

        end
%Question 2.2
        % update the position of robot i using its dynamics
        % X(i,k) = X(i,k-1);
        % Y(i,k) = Y(i,k-1); 
        % draw the robots as circles
        plot(X(i,k),Y(i,k),'oblack','LineWidth',3)
        xlabel('x')
        ylabel('y')
        title('Trajectories in real time')
        hold on
    end
    drawnow();
end

%% Figures
% 1st figure with the initial and final positions 
figure 
% plot the robots' initial states
plot(X(:,1),Y(:,1),'ored','LineWidth',3);
hold on
axis square;

% plot the robots' trajectories
for i=1:N
    plot(X(i,:),Y(i,:),'r') ;
end
% plot the robots' final states
for i=1:N
    plot(X(:,end),Y(:,end),'ogreen','LineWidth',3) ;
end
title('Robots trajectories') ; xlabel('position on x') ; ylabel('position on y') ;
% 2nd figure with the evolution of p_i(t) 
% you can plot 2 subfigures, one for each component of p_i
figure
% plot the robots' positions over time
axis square;
subplot(211) 
hold on 
for i=1:N
    plot(X(i,:)) ;
end
title('Positions X for the G1');xlabel('time'); ylabel('Position on X')
hold off
subplot(212) 
hold on 
for i=1:N
    plot(Y(i,:)) ;
end
title('Positions Y for the G1');xlabel('time'); ylabel('Position on Y')
hold off
