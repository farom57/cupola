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
  M*X-h
  sigma=sqrt(sumsq(M*X-h)/3/N);
  
endfunction