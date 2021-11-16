Xref = Vx*time;

% Desired Y position
z1 = (2.4/50)*(Xref-27.19)-1.2;
z2 = (2.4/43.9)*(Xref-56.46)-1.2;
Yref = 8.1/2*(1+tanh(z1)) - 11.4/2*(1+tanh(z2));

% Desired curvature
DX = gradient(Xref,0.1);
DY = gradient(Yref,0.1);
D2Y = gradient(DY,0.1);

curvature = DX.*D2Y./(DX.^2+DY.^2).^(3/2);

plot(time, curvature);