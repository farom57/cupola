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