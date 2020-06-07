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