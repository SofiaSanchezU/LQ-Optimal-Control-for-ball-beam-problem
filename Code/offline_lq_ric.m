function [Ft,Pt]=offline_lq_ric(A,B,Q,R,S,M,N);

% Let us implement the offline part of the LQ strategy
% I'll use a 3-D vector to implement the sequence of the P matrices

At=A-B*inv(R)*M;
Bt=B;
Qt=Q-M'*inv(R)*M;
Rt=R;
St=S;
Pt(:,:,N+1) = St; % P sequence initialization
% OFF-LINE part

for i=N:-1:1

   % Kalman Gains Computation

   Ft(i,:) = inv(Rt+Bt'*Pt(:,:,i+1)*Bt)*Bt'*Pt(:,:,i+1)*At;

   % P sequence updating phase (i'll use the riccati equation)

   Pt(:,:,i) = At'*Pt(:,:,i+1)*At+Qt-At'*Pt(:,:,i+1)*Bt*inv(Rt+Bt'*Pt(:,:,i+1)*Bt)*Bt'*Pt(:,:,i+1)*At;
   
   % Q + F(i,:)'*R*F(i,:) + (A-B*F(i,:))'*P(:,:,i+1)*(A-B*F(i,:));

end;    