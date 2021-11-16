function d = insulinGlucoseODE (t , y , u)

%% y= [ G; X; I ]

G = y(1,1); %% mmol/liter
X = y(2,1);
I = y(3,1);

%% Parameters 

p1 = 0.00;
p2 = 0.025;
p3= 0.000013;
VI = 12;
nn =5/54;
GB= 4.5;
IB=15;
B = 0.5;
k = 0.05;
    
%% Calculating inputs
Pt = B * exp( -k * t);

%% Calculating derivatives
d = zeros(3,1);

d(1,1) =  -p1 * G - X* (G+GB) + Pt;  %% Gdot = mmol/liter/min
d(2,1) = -p2 * X + p3 * I;
d(3,1) = -nn * (I+IB) + 1000*u / VI/60;

end