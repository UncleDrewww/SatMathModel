function y = EulerDot(u)
wx = u(1);
wy = u(2);
wz = u(3);
Phi = u(4);
Theta = u(5);
% Psi = u(6);

% y = [cos(Theta) sin(Phi)*sin(Theta) cos(Phi)*sin(Theta);...
%     0 cos(Phi)*cos(Theta) -sin(Phi)*cos(Theta);...
%     0 sin(Phi) cos(Phi)]/cos(Theta)*[wx;wy;wz];
y = [1 0 -sin(Theta);...
    0 cos(Phi) sin(Phi)*cos(Theta);...
    0 -sin(Phi) cos(Phi)*cos(Theta)]\[wx;wy;wz];