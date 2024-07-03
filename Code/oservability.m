
function Om = oservability (A, Cz)

Ra = rank(A);                      %rank of matrix A 

Om = obsv (A,Cz);                 %observability matrix

Ro = rank(Om);                    %Rank of O matrix

Om = Om;

