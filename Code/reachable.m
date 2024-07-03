

function Rm = reachable (A, B)

Ra = rank(A);                  %rank of A Matrix

Rm = ctrb(A, B);               %Reachability matrix

Rr = rank(Rm);                 %Rank of R

y = Rm;


