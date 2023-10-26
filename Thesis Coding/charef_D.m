
function [G_charef_D,omega_c,K_D,a,b,Z1,P1,N_D]=charef_D(alpha)


numerator1=1;
denominator1=1;

omega_l=0.001;
omega_h=1000;
omega_max=100*omega_h;

psi=10^-5;
y=1;

s=tf('s');

omega_c=omega_l*sqrt(10^(psi/(10*alpha))-1)
K_D=(omega_c^alpha)
%G_FPP= K_I/(1+s/omega_c)^alpha;

a=10^(y/(10*(1-alpha)))
b=10^(y/(10*alpha))

Z1=omega_c*10^(y/(20*alpha))
P1=a*Z1
N_D=round(log(omega_max/Z1)/log(a*b))+1

for i = 0:N_D
    numerator1 = numerator1 * (1+(s / (Z1 * (a*b)^i)));
end

% Calculate the denominator product
for i = 0:N_D
    denominator1 = denominator1 * (1+(s / (P1 * (a*b)^i)));
end

G_charef_D=(K_D*numerator1/denominator1);

end
