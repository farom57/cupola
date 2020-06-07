function ret=topo2dec(lat,ha,dec)
  topo2pole = rotx(pi/2-lat)
  pole2ha = rotz(pi/2+ha)
  ha2dec = roty(dec)

  ret = ha2dec*pole2ha*topo2pole