%DIGITAL CONTROL DESIGN TOOL
%
%This tool makes some useful digital filters for digitlal
%control applications. 

%Constants and Dependent Variables
T = 1/5;                    %sample period

fs = 1/T;                   %sample frequency

s = tf('s');                %frequency domain variable s

%1st LOWPASS FILTER
f1 = 3;                     %pole location
H1 = f1/(s+f1);             %Continuous time 1st order LPF
dH1 = c2d(H1,T,'tustin');   %Convert to discree time

%2nd LOWPASS FILTER
f2 = 3;                   %repeated pole location
H2 = f2^2/(s+f2)^2;         %Continuous time 2nd order LPF
dH2 = c2d(H2,T,'tustin');   %Convert to discree time

%USABLE DERIVATIVE
fd = 3;                     %Derivative cutoff freq
D = fd^2*s/(s+fd)^2;        %Continuous time filtered derivative
dD = c2d(D,T,'tustin');     %Convert to discree time

%PID COMPENSATOR:
%Controller gains
Kp = 10;
Ki = 1; 
Kd = 2;
%Compensator 
C = Kp + Ki/s + Kd*s*H1;    %Continuous Time PID compensator
dC = c2d(C,T,'matched');    %Convert to discrete by pz matching

%LEAD LAG COMPENSATOR:
%p1 and z1 are lead -- phase lead at high req, improves stability
%p2 and z2 are lag  -- phase lag at low freq, reduce ss error
%|p1| > |z1| > |z2| > |p2|
p1 = 3;
z1 = 2;
z2 = 1.1;
p2 = 1;

%Direct implementation
LL = (s+z1)*(s+z2)/((s+p1)*(s+p2));
dLL = c2d(LL,T,'tustin');

