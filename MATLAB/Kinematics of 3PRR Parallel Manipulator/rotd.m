%------- function r = rotd(n,phi); -----------------
% computes the rotation matrix for rotation about axis n phi degrees
%-----------------------------------------------------------
function r = rotd(n,phi);
  n = n ./ norm(n);
  phi=phi*pi/180; %convert to radians
  s = phi*[0 -n(3) n(2);n(3) 0 -n(1);-n(2) n(1) 0];
  r = expm(s);
return;
