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
g_theo=[0;0;9.81];
m_theo=m_theo./norm(m_theo);
g_theo=g_theo./norm(g_theo);

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
PN_PE=[mean(data(2090:2167,:),1) lat -pi/2 0];
N_PE=[mean(data(2299:end,:),1) lat -pi/2 lat];
data2=[S_PW;Z_PW;PN_PW;N_PW;PN;W;E;S_PE;Z_PE;PN_PE;N_PE]';

angles=data2(7:9,:);


[A_mag,bias_mag,sigma_mag]=compute_calibration(data2(1:3,:), angles, m_theo);
[A_acc,bias_acc,sigma_acc]=compute_calibration(data2(4:6,:), angles, g_theo);

mag_cal_tel = calibrate(data(:,1:3)',A_mag,bias_mag);
acc_cal_tel = calibrate(data(:,4:6)',A_acc,bias_acc);

mag2_cal_tel = calibrate(data2(1:3,:),A_mag,bias_mag);
acc2_cal_tel = calibrate(data2(4:6,:),A_acc,bias_acc);

for i=1:11
  mag2_theo_tel(:,i) = tel2topo(angles(1,i),angles(2,i),angles(3,i))'*m_theo;
  acc2_theo_tel(:,i) = tel2topo(angles(1,i),angles(2,i),angles(3,i))'*g_theo;
endfor

mag2_cal_topo = calibrate_and_rotate(data2(1:3,:),A_mag,bias_mag,angles);
acc2_cal_topo = calibrate_and_rotate(data2(4:6,:),A_acc,bias_acc,angles);

%figure(1),plot([mag2_cal_topo' acc2_cal_topo'])
sigma_acc
sigma_mag

%figure(2),plot3(mag_cal_tel(1,:),mag_cal_tel(2,:),mag_cal_tel(3,:),acc_cal_tel(1,:),acc_cal_tel(2,:),acc_cal_tel(3,:)),axis equal
[ha,dec,ha_rot,dec_rot,theta] = rot_estimate(mag2_cal_tel,acc2_cal_tel,lat,m_theo,sigma_acc,sigma_mag);

angles_bis = [ones(1,11)*lat;ha_rot;dec_rot];
[A_mag_bis,bias_mag_bis,sigma_mag_bis]=compute_calibration(data2(1:3,:), angles_bis, m_theo);
[A_acc_bis,bias_acc_bis,sigma_acc_bis]=compute_calibration(data2(4:6,:), angles_bis, g_theo);
sigma_acc_bis
sigma_mag_bis
mag_cal_tel_bis = calibrate(data(:,1:3)',A_mag_bis,bias_mag_bis);
acc_cal_tel_bis = calibrate(data(:,4:6)',A_acc_bis,bias_acc_bis);
mag2_cal_tel_bis = calibrate(data2(1:3,:),A_mag_bis,bias_mag_bis);
acc2_cal_tel_bis = calibrate(data2(4:6,:),A_acc_bis,bias_acc_bis);
[ha_bis,dec_bis,ha_rot_bis,dec_rot_bis,theta_bis] = rot_estimate(mag2_cal_tel_bis,acc2_cal_tel_bis,lat,m_theo,sigma_acc_bis,sigma_mag_bis);
figure(3),plot([ha_rot_bis'*360/2/pi angles_bis(2,:)'*360/2/pi],'+');
figure(4),plot([dec_rot_bis'*360/2/pi angles_bis(3,:)'*360/2/pi],'+');
figure(5),plot3(mag_cal_tel_bis(1,:),mag_cal_tel_bis(2,:),mag_cal_tel_bis(3,:),acc_cal_tel_bis(1,:),acc_cal_tel_bis(2,:),acc_cal_tel_bis(3,:)),axis equal
[ha_bis,dec_bis,ha_rot_bis,dec_rot_bis,theta_bis] = rot_estimate(mag_cal_tel_bis,acc_cal_tel_bis,lat,m_theo,sigma_acc_bis,sigma_mag_bis);
figure(6),plot([ha_rot_bis'*360/2/pi dec_rot_bis'*360/2/pi])
[(ha_rot_bis-angles_bis(2,:));(dec_rot_bis-angles_bis(3,:))]'*360/2/pi