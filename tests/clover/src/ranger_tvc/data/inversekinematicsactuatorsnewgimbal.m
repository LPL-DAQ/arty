clc;
clear;
close all;

%https://ntrs.nasa.gov/api/citations/20240012525/downloads/20240012525-rev01.pdf
%const values- same for both pitch yaw
x = 11.2120; %vertical dist from gimbal center to actuator mount height
te = 8.3981; %horizontal dist from center axis to actuator mount
Ltsy = sqrt(x^2+te^2); %length from gimbal center to actuator mount (const)
% actuator length at center = 18.3582 in
e = 6.6941; %length along engine axis from gimbal center to actuator mount on engine
er = 4.349; %length from center of engine center to actuator mount on engine
Lre = sqrt(e^2+er^2); %length from gimbal center to actuator mount on engine

%const angles
phi0 = 110.155; %degrees between Lts and Lre when engine centered;
psi0 = 113.696; %degrees between Lts and Lre when engine centered;

% Define the angles phi and yaw for calculations [change to table values
% later]
pitch = [-12:0.1:12]; 
yaw = [-12:0.1:12]; 

%L1 for pitch
%L2 for yaw
L1 = zeros(length(pitch),1);
L2 = zeros(length(yaw),1);
for ii = 1:length(pitch)
    L1(ii)= sqrt(Ltsy^2 + Lre^2 - 2 * Ltsy * Lre * cosd(phi0 + pitch(ii)));
end
for ii = 1:length(yaw)
    L2(ii)= sqrt(Ltsy^2 + Lre^2 - 2 * Ltsy * Lre * cosd(psi0 + yaw(ii)));
end
%Make into table
Table = cell(length(pitch),length(yaw));
for ii = 1:length(pitch)
    for jj = 1:length(yaw)
        Table{ii, jj} = [num2str(L1(ii)), ',', num2str(L2(jj))]; 
    end
end
p = pitch(:);
y = yaw(:);

L1T = table(p, L1,'VariableNames', {'pitch', 'L1'});
L2T = table(y, L2, 'VariableNames', {'yaw', 'L2'});
writetable(L1T,'TVCpitchnew.csv');
writetable(L2T,'TVCyawnew.csv');

