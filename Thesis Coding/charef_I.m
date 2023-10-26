
function [G_charef_I,omega_c,K_I,a,b,P0,Z0,N]=charef_I(alpha)


numerator=1;
denominator=1;

omega_l=0.001;
omega_h=1000;
omega_max=100*omega_h;

psi=10^-5;
y=1;

s=tf('s');

omega_c=omega_l*sqrt(10^(psi/(10*alpha))-1)
K_I=1/(omega_c^alpha)
%G_FPP= K_I/(1+s/omega_c)^alpha;

a=10^(y/(10*(1-alpha)))
b=10^(y/(10*alpha))

P0=omega_c*10^(y/(20*alpha))
Z0=a*P0
N=round(log(omega_max/P0)/log(a*b))+1

for i = 0:N-1
    numerator = numerator * (1+(s / (Z0 * (a*b)^i)));
end

% Calculate the denominator product
for i = 0:N
    denominator = denominator * (1+(s / (P0 * (a*b)^i)));
end

G_charef_I=(K_I*numerator/denominator);

end
