%Define the symbolic variable s
clc
s = fotf('s');
a=0;
b=0;

% Define the parameter alpha 

alpha_values = [0.1, 0.5, 0.9];
for j = 1:length(alpha_values)
    alpha = alpha_values(j);


% Define the frequency range
f = logspace(-3, 0, 100);  % [0.001, 1]

t=[0:1:1000]; % Adjust the time span as needed

omega=2*pi*f;



% Define the original transfer function O and application of the
% approximation methods (Charef C, Oustaloup OST)

O_I = 1/(s^alpha);  % I for Integrator
O_D=s^alpha;        % D for Derivative

C_I=charef_I(alpha);
C_D=charef_D(alpha);



OST_I=oustafod(-alpha,5,0.001,1000);
OST_D=oustafod(alpha,5,0.001,1000);




% ------------------------------------------------------------------

% Frequency Domain



% Calculate the magnitude and phase of the Original for the given frequencies
[magnitude_o_I, phase_o_I] = bode(O_I,omega);
[magnitude_o_D, phase_o_D] = bode(O_D,omega);


% Calculate the magnitude and phase of Charef approximation for the same frequencies
[magnitude_C_I, phase_C_I] = bode(C_I,omega);
[magnitude_C_D, phase_C_D] = bode(C_D,omega);


% Calculate the magnitude and phase of Oustaloup approximation for the same frequencies
[magnitude_Oust_I, phase_Oust_I] = bode(OST_I,omega);
[magnitude_Oust_D, phase_Oust_D] = bode(OST_D,omega);



% Calculate magnitude and phase errors for Charef approximation
magnitude_error_Charef_I =  magnitude_C_I(:)  -  magnitude_o_I(:) ;
phase_error_Charef_I = phase_C_I(:)  -  phase_o_I(:);

magnitude_error_Charef_D =  magnitude_C_D(:)  -  magnitude_o_D(:) ;
phase_error_Charef_D = phase_C_D(:)  -  phase_o_D(:);



% Calculate magnitude and phase errors for Oustaloup approximation
magnitude_error_Oustaloup_I = magnitude_Oust_I(:)-magnitude_o_I(:) ;
phase_error_Oustaloup_I = phase_Oust_I(:) - phase_o_I(:);

magnitude_error_Oustaloup_D = magnitude_Oust_D(:)-magnitude_o_D(:) ;
phase_error_Oustaloup_D = phase_Oust_D(:) - phase_o_D(:);



% Calculate the number of frequency points
num_points = length(f);

% Initialize an array to store the RME values

rme_charef_I = zeros(1, num_points);
rme_oustaloup_I = zeros(1, num_points);

rme_charef_D = zeros(1, num_points);
rme_oustaloup_D = zeros(1, num_points);


%--------------------------------------------------------------


%Magnitude RME

% Calculate RME for the FO Integrator for each frequency point
for i = 1:num_points
    A_I = magnitude_C_I(i);  % Magnitude of Charef approximation
    B_I=magnitude_Oust_I(i); % Magnitude of Oustaloup approximation
    R_I = magnitude_o_I(i);        % Magnitude of original system

    rme_charef_I(i) = ((A_I- R_I) / R_I )* 100;
    rme_oustaloup_I(i) = ((B_I - R_I) / R_I) * 100;
end


% Calculate RME for the FO Derivative for each frequency point
for i = 1:num_points
    A_D = magnitude_C_D(i);  % Magnitude of Charef approximation
    B_D=magnitude_Oust_D(i); % Magnitude of Oustaloup approximation
    R_D = magnitude_o_D(i);        % Magnitude of original system

    rme_charef_D(i) = ((A_D- R_D) / R_D )* 100;
    rme_oustaloup_D(i) = ((B_D - R_D) / R_D) * 100;
end



% plot the RME values as needed
rme_charef_I;
rme_oustaloup_I;

rme_charef_D;
rme_oustaloup_D;






%---------------------------------------------






b=b+1;

figure(a+1)
subplot(3, 1, b); 
bode(O_I,omega)
hold on
bode(OST_I,'r',omega)
hold on
bode(C_I,'g',omega)
title(['Bode diagram FO-Integrator for \alpha = ', num2str(alpha)]);



figure(a+2)
subplot(3, 1, b); 
bode(O_D,omega)
hold on
bode(OST_D,'r',omega)
hold on
bode(C_D,'g',omega)
title(['Bode diagram FO-Derivative for \alpha = ', num2str(alpha)]);




%-------------------------------------------------------

%Time Domain

% Define the unit step input
u = ones(size(t));

% Simulate the system's response to the unit step input
response_O_I = lsim(O_I, u, t);
response_C_I = lsim(C_I, u, t);
response_OST_I = lsim(OST_I, u, t);

response_O_D = lsim(O_D, u, t);
response_C_D = lsim(C_D, u, t);
response_OST_D = lsim(OST_D, u, t);


% Plot the response
figure(a+3);
subplot(3, 1, b); 
plot(t, response_O_I,'b');
hold on
plot(t, response_C_I,'g');
hold on
plot(t, response_OST_I,'r');
title(['Unit Step Response of FO Integrator (\alpha = ', num2str(alpha), ')']);
xlabel('Time');
ylabel('Output');
grid on;



%----------------------------------------------------

figure(a+4);
subplot(3, 1, b); 
plot(t, response_O_D,'b');
hold on
plot(t, response_C_D,'g');
hold on
plot(t, response_OST_D,'r');

title(['Unit Step Response of FO Derivative (\alpha = ', num2str(alpha), ')']);
xlabel('Time');
ylabel('Output');
grid on;





end


%--------------------------------------------------------

%Closed loop control system

%Motor Generator system

num=[0.63606];
den=[318.64 50.22 1];
sys=tf(num,den,'InputDelay',0.61)

% Using the FOPID Parameters

C_CLCS_I=charef_I(0.89);
C_CLCS_D=charef_D(0.44);



OST_CLCS_I=oustafod(-0.89,5,0.001,1000);
OST_CLCS_D=oustafod(0.44,5,0.001,1000);
