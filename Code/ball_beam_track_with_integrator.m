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

% Number of states and inputs 

[nx,nu]=size(Bc);

%% Sampling time

%abs(eig(Ac))etc etc 

Ts=0.0865;

% Number of steps

tfin=8;

N=ceil(tfin/Ts);

%% Tracking Problem
% Matrix Output to be tracked  (i wanna track the ball position, first state)

Cy=[1 0 0 0];

% Exosystem characterization (i'll start from its continuous time 
% counterpart)

Arc=0;  % I wanna generate a step

Cr=1;  
% I'll compute the discretized exosystem by recalling that
% Ar = expm(Arc*Ts)

Ar = expm(Arc*Ts);

[nr,nr]=size(Ar);   % number of states of the exosystem

% Let us compute the discretized plant

ball_beam_tc=ss(Ac,Bc,Cc,Dc);

ball_beam_td=c2d(ball_beam_tc,Ts);

% Extract the matrices

[A00,B00,C00,D00]=ssdata(ball_beam_td);
% Augmented System

% Augmented System with the adder input structure

A0=[A00 B00;
    zeros(nu,nx) eye(nu)];

B0=[B00;eye(nu)];

% Complete augmented system with the exosystem (in this case the step
% reference generator

A=blkdiag(A0,Ar);
B=[B0;zeros(nr,nu)];

% Weights of the LQ tracking performance index

Qy=1;
Sy=1;

Q=[Cy';zeros(nu,nr);-Cr']*Qy*[Cy zeros(nr,nu) -Cr];
S=[Cy';zeros(nu,nr);-Cr']*Sy*[Cy zeros(nr,nu) -Cr];

% Tuning parameter rho

sqrtrho=.018;

R=sqrtrho*sqrtrho*eye(nu);

% Mixed term matrix

M=zeros(nu,nx+nu+nr);

% Offline Riccati Backward Iterations

[Fint,Pint]=offline_lq_ric(A,B,Q,R,S,M,N);


%% Online part
% Vectors collecting the state, input and time trend taken from Simulink

xseq=[];

useq=[];

tseq=[];

timesamp=[];

ref_seq=[];

% Initial state which depicts a perturbation w.r.t. to the equilibrium

dx0=[0.05;0.1;0;0];

xInitial=xeq+dx0;

% I'll consider the suboptimal strategy

F=Fint(1,1:nx);         %  Feedback component

Fu=Fint(1,nx+1:nx+nu);  %  Adder input component

Fv=Fint(1,nx+nu+1:end); % Feedforward component

u_adder=zeros(nu,1);    % Intial value of the adder


xrk=0.2; % 20 cm upwards at the end of the transient

for i=1:N;

  t=(i-1)*Ts;
  
  Deltau=-F*(xInitial-xeq)-Fu*u_adder-Fv*xrk;  % incremental input as sum of feedback 
                                  % and feeforward components

  u=(ueq+u_adder+Deltau)';             % control law in an adder fashion u_k=u_{k-1} + Delta u_k
                                     % and feeforward components
                                     % Input Bias + Control Strategy

  result=sim("ball_beam_sim");

  xseq=[xseq result.xout'];

  useq=[useq u(1).*ones(nu,length(result.tout))];

  tseq=[tseq result.tout'];
  
  ref_seq=[ref_seq Cr*xrk+xeq(1)]; % Absolute reference (end position)

  timesamp=[timesamp (i-1)*Ts];

  xInitial=result.xFinal';
  
  % Signal to be tracked update

  xrk=Ar*xrk;
  
  % Adder update  u_k = u_{k-1} + Delta u_k

  u_adder = u_adder+Deltau;

end;    
%subplot(3,1,1);
plot(tseq,xseq(1,:),timesamp,ref_seq,'LineWidth',2);
grid;
title('Position');
% subplot(3,1,2);
% plot(tseq,xseq(3,:),'LineWidth',2);
% grid;
% title('Angular Displacement');
% subplot(3,1,3);
% plot(tseq,useq,'LineWidth',2);
% grid;
% title('Force');

