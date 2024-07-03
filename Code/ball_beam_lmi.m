clc
close all
clear all
%% Parameters of the real system: [I, m, J, R, g, L]
parameters=[0.8, 1.5, 2.5*10^-2, 0.08, 9.81, 1]; 
%initial state condition 
x0 = [0,0,0,0];

%% Trim code
%%Specify the model name
model = 'ball_beam_trim';
opspec = operspec(model);
%%Set the constraints on the states in the model.
% State (1) - ball_beam_trim/S-Function Builder
opspec.States(1).x = [0.1;0;0;0];
opspec.States(1).Known = [true;true;true;true];

%%Set the constraints on the inputs in the model.
% Input (1) - ball_beam_trim/In1
opspec.Inputs(1).u = 0.5216;

%%Create the options
opt = findopOptions('DisplayReport','iter');

%%Perform the operating point search.
[op,opreport] = findop(model,opspec,opt);

%% Extract the equilibrium
xeq=op.States.x;
ueq=op.Inputs.u;

%% Compute the linearized plant using linmod

[Ac,Bc,Cc,Dc]=linmod('ball_beam_trim',xeq,ueq);

%% Sampling time

%abs(eig(Ac))etc etc 

Ts=0.0865;

% Plant dimensions, instrumental for LQ regulation

[nx,nu]=size(Bc);

% Let us compute the discretized plant

ball_beam_tc=ss(Ac,Bc,Cc,Dc);

ball_beam_td=c2d(ball_beam_tc,Ts);

% Extract the matrices

[A,B,C,D]=ssdata(ball_beam_td);

%% Define the LQ parameters Q, R and M
Cz=[1 0 0 0;
    0 0 1 0;
    0 0 0 0];

[nz,nx]=size(Cz);

sqrtrho=.05;

Dzu=sqrtrho*[0;0;1];

Q=Cz'*Cz;

S=Q;

R=Dzu'*Dzu;

M=zeros(nu,nx);

% Number of steps

tfin=4;

N=ceil(tfin/Ts);

%% Online part
% Vectors collecting the state, input and time trend taken from Simulink

xseq=[];

useq=[];

tseq=[];

% Initial state which depicts a perturbation w.r.t. to the equilibrium

dx0=[0.05;0.1;0;0];

xInitial=xeq+dx0;


%% LMI Section

ops=sdpsettings('verbose',0);

% LMI variables alpha, Y, K

alpha=sdpvar(1);

Y=sdpvar(nx);

K=sdpvar(nu,nx);


% LMI constraints


% Normalized Upper bound


V1=[1 dx0';
    dx0 Y] >= 0;


% Y positive

V2=Y >= 0;

% Riccati discrete time inequality (LMI version)


V3 = [Y Y*A'-K'*B' Y*Cz'-K'*Dzu';
      A*Y-B*K  Y zeros(nx,nz);
      Cz*Y-Dzu*K zeros(nz,nx) alpha*eye(nz)] >= 0;

% Constraints alltogether

V=[V1,V2,V3];


% Optimze

lmi_result=optimize(V,alpha,ops);

% F and X (original variables)

F_lmi = double(K)*inv(double(Y));

X_lmi = double(alpha)*inv(double(Y));

tic
for i=1:N;

  % Current Input move

  u=[-F_lmi*(xInitial-xeq)+ueq(1)]';

  % Current sampling time

  t=(i-1)*Ts;

  result=sim("ball_beam_sim");

  xseq=[xseq result.xout'];

  useq=[useq u(1).*ones(1,length(result.tout))];

  tseq=[tseq result.tout'];

  xInitial=result.xFinal';

end;    
toc
figure 1
subplot(3,1,1);
plot(tseq,xseq(1,:),'LineWidth',2);
grid;
title('Position');
subplot(3,1,2);
plot(tseq,xseq(3,:),'LineWidth',2);
grid;
title('Angular Displacement');
subplot(3,1,3);
plot(tseq,useq,'LineWidth',2);
grid;
title('Force');