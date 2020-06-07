close all
clear all

% O = Origin
% A = Intersection axes RA/DEC
% B = Point sur l'axe de l'instrument

% n = vecteur vers le nord geometrique
% s = vecteur vers le sud geometrique
% p = vecteur vers le pole nord celeste
% d = vecteur position moyenne du soleil à midi
% m = champ magnetique
% g = -z = nadir
% w = vecteur vers l'ouest
% z = vecteur vers le zenith
% c = du contre-poids vers l'instrument
% i = instrument vers la cible
% _ = vecteur perpendiculaire aux 2 autres pour formerune base orthonormée directe

% ha_rot rotation of the ra axis in rad, 0 = vertical with the counterweights down, pi/2 = counterweight pointing west = telescope usualy pointing east
% dec_rot rotation of the dec axis in rad, 0 = telescope pointing north pole, pi/2 = pointing west when ha_rot=0

% topo = wsz = [ouest sud zenith]
%   rotation d'axe x et d'angle pi/2-lat
% pole = wdp = [ouest midi pole_nord]
%   rotation d'axe z et d'angle ha_rot
% ha = _cp = [_|_ CP2instrum pone_nord]
%   rotation d'axe y et d'angle dec_rot
% tel = _ci = [_|_ CP2instrum target]

%% Constantes
lat=43.56*2*pi/360;
m_theo=[-0.9983;-23.7225;-40.3228]; % https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
m_theo=m_theo./norm(m_theo)
g_theo=[0;0;1];

%% Preparations des données
data=dlmread('data_22052020_3.tsv');
data=data(:,1:6);
it=1:size(data,1);

%% Etiquetage des données
%% 9 pointages, S=sud E=Est W=ouest Z=zenith PN=pole nord PE=contrepoint à l'est PW= contrepoind à l'ouest
%% Angles: latitude ha_rot dec_rot
S_PW=[mean(data(1:84,:),1) lat pi/2 pi-lat]; 
Z_PW=[mean(data(162:300,:),1) lat pi/2 pi/2-lat]; 
PN_PW=[mean(data(466:632,:),1) lat pi/2 0]; 
N_PW=[mean(data(672:828,:),1) lat pi/2 -lat]; 
PN=[mean(data(916:1031,:),1) lat 0 0];
W=[mean(data(1122:1267,:),1) lat 0 pi/2];
E=[mean(data(1382:1511,:),1) lat 0 -pi/2];
S_PE=[mean(data(1633:1724,:),1) lat -pi/2 lat-pi];
Z_PE=[mean(data(1829:1944,:),1) lat -pi/2 lat-pi/2];
PN_PW=[mean(data(2090:2167,:),1) lat -pi/2 0];
N_PW=[mean(data(2299:end,:),1) lat -pi/2 lat];
data2=[S_PW;Z_PW;PN_PW;N_PW;PN;W;E;S_PE;Z_PE;PN_PW;N_PW]';


[A_mag,bias_mag,sigma_mag]=compute_calibration(data2(1:3,:), data2(7:9,:), m_theo)
[A_acc,bias_acc,sigma_acc]=compute_calibration(data2(4:6,:), data2(7:9,:), g_theo)

mag_cal_tel = calibrate(data2(1:3,:),A_mag,bias_mag);
acc_cal_tel = calibrate(data2(4:6,:),A_acc,bias_acc);

mag_cal_topo = calibrate_and_rotate(data2(1:3,:),A_mag,bias_mag,data2(7:9,:));
acc_cal_topo = calibrate_and_rotate(data2(4:6,:),A_acc,bias_acc,data2(7:9,:));

plot([mag_cal_topo' acc_cal_topo'])



