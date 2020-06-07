%% calibre les mesures
%% raw (3xN)
%% calibrated (3xN)
function calibrated_topo=calibrate_and_rotate(raw,A,bias,angles)
  A_=inv(A);
  N=size(raw,2);


  for i=1:N
    calibrated(:,i)=A_*(raw(:,i)-bias);
    calibrated_topo(:,i)=tel2topo(angles(1,i),angles(2,i),angles(3,i))*calibrated(:,i);
  endfor
endfunction