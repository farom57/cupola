% mag et acc  sont donnés dans le repere tel = [_|_ CP2instrum target]
% mag_ref est donné dans le repere topo = wsz = [ouest sud zenith]
function [ha,dec,ha_rot,dec_rot,theta] = rot_estimate(mag,acc,lat,mag_ref,sigma_acc,sigma_mag)
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
  if dec_rot>=0
    dec=pi/2-dec_rot;
    ha=ha_rot+pi/2;
  else
    dec=pi/2+dec_rot;
    ha=ha_rot-pi/2;
  endif
  
endfunction
