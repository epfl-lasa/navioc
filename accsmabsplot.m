Ax = (-0.5:0.01:0.5)';
Ay = Ax*0;
A2 = Ax.^2 + Ay.^2;
Z = sqrt(A2);

s = 10;

Expz = exp(-2*s*Z);
F = Z + 1/s*log(1 + Expz) - 1/s*log(2);
plot(Ax, Z, 'k', Ax, Ax.^2, 'k--', Ax, F, 'r')