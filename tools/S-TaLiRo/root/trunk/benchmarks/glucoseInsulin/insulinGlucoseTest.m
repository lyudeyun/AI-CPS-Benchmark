G0 = 10.5;
X0 = 0.08;
I0 = 0;

%%inpMatrix = [ 0 457; 0.01 0; 120 0.8; 180 1.4; 210 0.8; 240 1.3 ]; %%
%%injection + schedule

inpMatrix = [ 0 457; 0.01 1]; %% injection + basal
%%inpMatrix=[0.01 1; 1 1]; %% purely basal

global inpMatrix;

[T,X] = ode45(@insulinGlucoseODE, [0 360], [G0; X0; I0] );

subplot(3,1,1);
plot(T/60,X(:,1))
title('Insulin Concentration with Time');
subplot(3,1,2);
plot(T/60,X(:,2));
title('Variation of X with Time');
subplot(3,1,3);
plot(T/60,X(:,3));
title('Insulin Concentration with Time');

