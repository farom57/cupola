% Code pour la calibration de la monture à implementer en Arduino

1;

%% Matrices de rotation
function ret=rotx (a)
ret=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
endfunction
function ret=roty (a)
ret=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];
endfunction
function ret=rotz (a)
ret=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
endfunction
function _ci_wsz=tel2topo(lat,ha_rot,dec_rot)
% topo = wsz = [ouest sud zenith]
%   rotation d'axe x et d'angle pi/2-lat
% pole = wdp = [ouest midi pole_nord]
%   rotation d'axe z et d'angle ha_rot
% ha = _cp = [_|_ CP2instrum pone_nord]
%   rotation d'axe y et d'angle dec_rot
% tel = _ci = [_|_ CP2instrum target]

wdp_wsz=rotx(pi/2-lat);
_cp_wdp=rotz(ha_rot);
_ci__cp=roty(dec_rot);
_ci_wsz=wdp_wsz*_cp_wdp*_ci__cp;
endfunction

%% calibre les mesures
%% raw (3xN)
%% calibrated (3xN)
function calibrated=calibrate(raw,A,bias)
  A_=inv(A);
  N=size(raw,2);

  for i=1:N
    calibrated(:,i)=A_*(raw(:,i)-bias);
  endfor
endfunction

%% Calcule la matrice de transformation A et le biais bias pour corriger les mesures
%% measurements = mesures dans le referentiel du capteur 3xN
%% angles = [latitude ha_rot dec_rot] 3xN
%% theory = Valeur théorique du champ magnetique ou de l'acceleration 3xN (dans lerepere topo)
%% measurment = A * theory_tel + bias + residus
function [A,bias,sigma]=compute_calibration(measurements, angles, theory)
  N=size(measurements,2);
  M=zeros(3*N,12);
  h=measurements(:);
  for i=1:N
    F(:,i)=tel2topo(angles(1,i),angles(2,i),angles(3,i))'*theory; %% vecteur théorie dans le repere tel
    M(i*3-2,:)=[F(1,i) 0 0 F(2,i) 0 0 F(3,i) 0 0 1 0 0];
    M(i*3-1,:)=[0 F(1,i) 0 0 F(2,i) 0 0 F(3,i) 0 0 1 0];
    M(i*3-0,:)=[0 0 F(1,i) 0 0 F(2,i) 0 0 F(3,i) 0 0 1];
  endfor
  
  X=pinv(M)*h;
  A=[X(1) X(2) X(3);X(4) X(5) X(6);X(7) X(8) X(9)]';
  bias=[X(10);X(11);X(12)];

  error=calibrate(measurements,A,bias)-F;
  sigma=sqrt(sumsq(error(:))/3/N);
  
endfunction

% mag et acc  sont donnés dans le repere tel = [_|_ CP2instrum target]
% mag_ref est donné dans le repere topo = wsz = [ouest sud zenith]
function [ha_rot,dec_rot] = rot_estimate(mag,acc,lat,mag_ref,sigma_acc,sigma_mag)
  a=mag_ref(2);
  b=mag_ref(1);
  c=mag_ref(3);
  acc=acc./norm(acc,"cols");;
  mag;
  acc;
  
  D2=(sigma_acc./cos(lat)).^2;
  T2=(sigma_mag./a).^2;
  
  % Calcul de l'angle horaire
  % en théorie cs = cos(ha_rot) et sn = sin(ha_rot) 
  cs = acc(2,:)./cos(lat);
  sn = 1./a.*(mag(1,:).*acc(3,:)-mag(3,:).*acc(1,:)-b.*sin(lat).*cs);
  
  % en pratique les erreurs de mesures font qu'il y a une incertitude sigma=sqrt(D2) sur cs et sigma=sqrt(T2) sur sn
  % la solution optimale minimise cos(ha_rot)-cs)²/D² + sin(ha_rot)-sn)²/T²
  % resolution iterative:  
  ha_rot=atan2(sn,cs);
  for i=1:5
    delta=T2.*sin(ha_rot).*(cos(ha_rot)-cs)-D2.*(sin(ha_rot)-sn);
    ddelta_dha=(T2-D2).*(cos(ha_rot).^2-sin(ha_rot).^2)-T2.*cos(ha_rot).*cs-D2.*sin(ha_rot).*sn;
    ha_rot=ha_rot-delta./ddelta_dha;
  endfor
  
  theta = atan2(cos(lat).*sin(ha_rot),sin(lat));
  dec_rot = theta - atan2(acc(1,:),acc(3,:));
 
endfunction

% Parametres BLE à creer
lat=43.56;
m_theo=[-0.9983;-23.7225;-40.3228]; % https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm

lat=deg2rad(lat);
g_theo=[0;0;9.81];
m_theo=m_theo./norm(m_theo);
g_theo=g_theo./norm(g_theo);


% A remplacer par acquisition durant 1s a chaque appuisur le bouton
% On doit obtenir sample_mag et sample_acc de taille 3x11
data=dlmread('data_22052020_3.tsv');
data=data(:,1:6);
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
%angles=data2(7:9,:);
sample_mag=data2(1:3,:);
sample_acc=data2(4:6,:);

angles=[
lat lat lat lat lat lat lat lat lat lat lat;
pi/2 pi/2 pi/2 pi/2 0 0 0 -pi/2 -pi/2 -pi/2 -pi/2;
pi-lat pi/2-lat 0 -lat 0 pi/2 -pi/2 lat-pi lat-pi/2 0 lat];

% first calibration using theorical angles
[A_mag,bias_mag,sigma_mag]=compute_calibration(sample_mag, angles, m_theo);
[A_acc,bias_acc,sigma_acc]=compute_calibration(sample_acc, angles, g_theo);
sigma_mag
sigma_acc

% Adjust angles
sample_mag_cal = calibrate(sample_mag,A_mag,bias_mag);
sample_acc_cal = calibrate(sample_acc,A_acc,bias_acc);
[ha_rot,dec_rot]=rot_estimate(sample_mag_cal,sample_acc_cal,lat,m_theo,sigma_acc,sigma_mag);
ha_max_error=max(abs(ha_rot-angles(2,:)))*360/2/pi
dec_max_error=max(abs(dec_rot-angles(3,:)))*360/2/pi
angles=[angles(1,:);ha_rot;dec_rot];

% second calibration using corrected angles
[A_mag,bias_mag,sigma_mag]=compute_calibration(sample_mag, angles, m_theo);
[A_acc,bias_acc,sigma_acc]=compute_calibration(sample_acc, angles, g_theo);
sigma_mag
sigma_acc

% Final check
sample_mag_cal = calibrate(sample_mag,A_mag,bias_mag);
sample_acc_cal = calibrate(sample_acc,A_acc,bias_acc);
[ha_rot,dec_rot]=rot_estimate(sample_mag_cal,sample_acc_cal,lat,m_theo,sigma_acc,sigma_mag);
ha_max_error=max(abs(ha_rot-angles(2,:)))*360/2/pi
dec_max_error=max(abs(dec_rot-angles(3,:)))*360/2/pi
angles=[angles(1,:);ha_rot;dec_rot];