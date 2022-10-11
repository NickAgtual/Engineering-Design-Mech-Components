clear; close all; clc
%% Difining Known values
% Tangential force lbf
I4.Wt = 786.27;
I5.Wt = 786.27;
                    
% Angle from horizontal axis
I4.angle = 48.18;
I5.angle = 167.52;

% Pressure angle
phi = 20;

% Gearbox total width 
gearboxWidth = 3.82;

% Shaft Length
shaftLength = 3.32;

% Bearing width 
widthBearing = .375;
diameterBearing = .375;

% Updated Shaft Length
shaftLeghtBearingToBearing = shaftLength - widthBearing;

% Location of load (gear) on shaft
loadPosition = 2.25;
faceWidth = 1.1;

%% Reaction forces on shaft
%Defining symbolic variables
syms Rx Ry

% Sum of forces in the x-direction
% Assuming positive reaction
sumForcesX = Rx - (I5.Wt * tand(phi) * cosd(I5.angle)) + ...
                  (I4.Wt * tand(phi) * cosd(I4.angle)) + ...
                  (I5.Wt * sind(I5.angle)) - ...
                  (I4.Wt * sind(I4.angle)) == 0;
              
% Solving for the reaction force in the x-direction             
RxShaft = solve(sumForcesX, Rx); 

% Sum of forces in the y-direction
% Assuming positive reaction
sumForcesY = Ry - (I5.Wt * tand(phi) * sind(I5.angle)) + ...
                  (I4.Wt * tand(phi) * sind(I4.angle)) - ...
                  (I5.Wt * cosd(I5.angle)) + ...
                  (I4.Wt * cosd(I4.angle)) == 0;
              
% Solving for the reaction force in the y-direction
RyShaft = solve(sumForcesY, Ry);

%% Defining equillibrium eqiations for the shaft

% Definign symbolic variables
syms RAx RAy RBx RBy

% Construction force and position vectors
% Assuming point A to be the origin
% [i j k]
reactionA.force = [RAx RAy 0];
reactionA.position = [0 0 0];

reactionB.force = [RBx RBy 0];
reactionB.position = [0 0 shaftLeghtBearingToBearing];

gear3.force = [RxShaft RyShaft 0];
gear3.position = [0 0 loadPosition];

% Sum of forces in the x-direction
% Assuming positive reaction

sumForcesXShaft = reactionA.force(1) + ...
                  reactionB.force(1) + ...
                  gear3.force(1) == 0;


% Sum of forces in the y-direction
% Assuming positive reaction

sumForcesYShaft = reactionA.force(2) + ...
                  reactionB.force(2) + ...
                  gear3.force(2) == 0;

% Moment equations
sumMomentsShaft = cross(reactionA.position, reactionA.force) + ...
                  cross(reactionB.position, reactionB.force) + ...
                  cross(gear3.position, gear3.force) == 0;
              
% Creating matrices for system of equations
[A, b] = equationsToMatrix(sumForcesXShaft, ...
                           sumForcesYShaft, ...
                           sumMomentsShaft, ...
                           [RAx RAy RBx RBy]);
                       
% Solving system of linear equations                    
reactionForces = linsolve(A, b);

%% Shear and Bending Moment Diagrams

% ----- y-z plane -----

% Creating new figure
figure(1)

% Shear diagram
subplot(2, 2, 1)

syms x

shearYZ = piecewise(x < 0, 0, 0 < x < loadPosition, ...
          reactionForces(2, 1), loadPosition < x < ...
          shaftLeghtBearingToBearing, reactionForces(2, 1) + RyShaft, ...
          x > shaftLeghtBearingToBearing, 0);

% Plotting shear  diagram 
fplot(shearYZ);

% Plot parameters
hold on
grid on
grid minor

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, x > shaftLeghtBearingToBearing, 0);

% Plotting while lines
whitePlot = fplot(white, 'Color', [1 1 1]);

% Turning asymptotes off
whitePlot.ShowPoles = 'Off';

% Plotting y = 0
yline(0, 'k')

% Setting axis limits
xlim([-.25, 3.5]);
ylim([-1200 1100]);

% Axis Descriptors
xlabel('\emph {z - distance (in)}', ...
    'fontsize', 12, 'Interpreter', 'latex');
ylabel('\emph {V (lbf)}', 'fontsize', 12, 'Interpreter', 'latex');
title('\emph {Shear Diagram (y-z Plane)}', 'fontsize', ...
14, 'Interpreter', 'latex');

% Moment diagram
subplot(2, 2, 2)

syms b

% Equation solving for y-intercept
interceptEq = (reactionForces(2,1) * loadPosition) == ...
    (-reactionForces(4,1) * loadPosition) + b;

% Solving for y=intercept
b = double(solve(interceptEq, b));

% Piecewsie moment equations
momentEq1 = reactionForces(2, 1) * x;
momentEq2 = (-reactionForces(4,1) * x) + b;

% Combined piecewise equation
momentYZ = piecewise(x < 0, 0, 0 < x < loadPosition, momentEq1, ...
           loadPosition < x < shaftLeghtBearingToBearing, momentEq2, ...
           x > shaftLeghtBearingToBearing, 0);

% Plotting moment diagram
fplot(momentYZ, 'r')

% Setting axis limits
xlim([-.25, 3.5]);
ylim([-1200 1100]);

% Plot parameters
hold on
grid on
grid minor

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, x > shaftLeghtBearingToBearing, 0);

% Plotting while lines
whitePlot = fplot(white, 'Color', [1 1 1]);

% Turning asymptotes off
whitePlot.ShowPoles = 'Off';

% Plotting y = 0
yline(0, 'k')

% Axis Descriptors
xlabel('\emph {z - distance (in)}', ...
    'fontsize', 12, 'Interpreter', 'latex');
ylabel('\emph {V (lbf)}', 'fontsize', 12, 'Interpreter', 'latex');
title('\emph {Moment Diagram (y-z Plane)}', 'fontsize', ...
14, 'Interpreter', 'latex');

% ----- x-y plane -----

% Shear diagram
subplot(2, 2, 3)

% Defining new symbolic variabel
syms x1 

% Piecewise function
shearXY = piecewise(x1 < 0, 0, 0 < x1 < loadPosition, ...
    reactionForces(1, 1), loadPosition < x1 < shaftLeghtBearingToBearing,  ...
    reactionForces(1, 1) + RxShaft, x1 > shaftLeghtBearingToBearing, 0);

% Plotting shear diagram
fplot(shearXY)

% Plot parameters
hold on
grid on
grid minor

% Axis Limits
xlim([-.25, 3.5]);
ylim([-50 50]);

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, x > shaftLeghtBearingToBearing, 0);

% Plotting while lines
whitePlot = fplot(white, 'Color', [1 1 1]);

% Turning asymptotes off
whitePlot.ShowPoles = 'Off';

% Plotting y = 0
yline(0, 'k')

% Axis Descriptors
xlabel('\emph {z - distance (in)}', ...
    'fontsize', 12, 'Interpreter', 'latex');
ylabel('\emph {V (lbf)}', 'fontsize', 12, 'Interpreter', 'latex');
title('\emph {Shear Diagram (x-z Plane)}', 'fontsize', ...
14, 'Interpreter', 'latex');

% Moment diagram 
subplot(2, 2, 4)

syms b2

% Equation solving for y-intercept
interceptEq2 = (reactionForces(1,1) * loadPosition) == ...
    (-reactionForces(3,1) * loadPosition) + b2;

% Solving for y=intercept
b2 = double(solve(interceptEq2, b2));

% Piecewsie moment equations
moment2Eq1 = reactionForces(1,1) * x1;
moment2Eq2 = (-reactionForces(3,1) * x1) + b2;

% Combined piecewise function
momentXY = piecewise(x1 < 0, 0, 0 < x1 < loadPosition, moment2Eq1, ...
           loadPosition < x1 < shaftLeghtBearingToBearing, moment2Eq2, ...
           x1 > shaftLeghtBearingToBearing, 0);

% Plotting moment diagram
fplot(momentXY, 'r');

% Plot parameters
hold on
grid on
grid minor

% Setting axis limits
xlim([-.25, 3.5]);
ylim([-50 50]);

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, x > shaftLeghtBearingToBearing, 0);

% Plotting while lines
whitePlot = fplot(white, 'Color', [1 1 1]);

% Turning asymptotes off
whitePlot.ShowPoles = 'Off';  

% Plotting y = 0
yline(0, 'k')

% Axis Descriptors
xlabel('\emph {z - distance (in)}', ...
    'fontsize', 12, 'Interpreter', 'latex');
ylabel('\emph {V (lbf)}', 'fontsize', 12, 'Interpreter', 'latex');
title('\emph {Moment Diagram (x-z Plane)}', 'fontsize', ...
14, 'Interpreter', 'latex');

% Subplot title
sgtitle('Hydraulic Pump Drive Shaft', 'fontsize', 16, 'Interpreter', ...
    'latex');


%% Shaft Stress Analysis

% Shaft diameter
d = [diameterBearing .5 .41 .41 .41];
% Elements 3 & 4 are for the keway and snap ring and will be replaced

% Neighboring diameter (Larger one)
D = [.5 .5 .41 .41 .41];
% Elements 3 & 4 are for the keway and snap ring and will be replaced

% Critical loactions
xCritical = [diameterBearing/2 (loadPosition - (faceWidth / 2)) loadPosition ...
            (loadPosition + (faceWidth / 2))  ...
            shaftLeghtBearingToBearing - diameterBearing/2];

% All factors use the following format if necessary:
% factor = [bendingFactor torsionFactor]

%  4142 Q&T 600F  Steel 
Sut = 280; % Ulitmate strength kpsi
Sy = 250; % Yield strength kpsi

SeP = .5 * Sut;

% ----- Endurance Limit -----
% Se = (ka)(kb)(kc)(kd)(ke)(kf)(Se')

% Surface factor (ka)
a = 2.7; % Machined or cold-drawn
b = -.265; % Machined or cold-drawn
ka = (a * (Sut ^ b));

% Size factor (kb)

kb = zeros(1, length(d));

for ii = 1:length(d)
    
    kb(ii) = .879 * (d(ii)^-.108);
    
end

% Loading factor (kc)
kc = [1 .59];

% Temperature factor (kd)
Tf = 250; % max operating temp in deg F
kd = .975 + (.432e-3 * Tf) - (.115e-5 * (Tf^2)) + (.104e-8 * (Tf^3)) - ...
    (.595e-12 * (Tf^4)); % 70 <= Tf <= 1000 def F

% Reliability Factor (ke)
%reliabilityPercent = [50 90 95 99 99.9 99.99 99.999 99.9999]
% reliabilityFactor = [1 .897 .868 .814 .753 .702 .659 .62]

% Reliability Factor
ke = (((98 - 95) / (99 - 95)) * (.814 - .868)) + .868;

% kf
kf = 1;

% Endurace limit

Se = zeros(1, length(d));

for ii = 1:length(d)
    
Se(ii) = ka * kb(ii) * kc(1) * kd * ke * kf * SeP;

end

% ----- Reduced Stress Concentration Factor for Bending -----

% Convention:
% Values at critical locations work from left to right 
% Left = front of gearbox
% Right = back of gearbox

% Neuber Constnat Bending (sqrt(a))
neuberBending = .246 - (3.08e-3 * Sut) + (1.51e-5 * (Sut^2)) - ...
                (2.67e-8 * (Sut^3));
            
% Geometric stress-concentration factor (Kt)

t = zeros(1, length(d));

for ii = 1:length(d)
    
    t(ii) = D(ii) / (2 * d(ii));
    
end

r = [.1 .05 .1 .1 .1];

% Initializing C factor vectors
C1Bending = zeros(1, length(t));
C2Bending = zeros(1, length(t));
C3Bending = zeros(1, length(t));
C4Bending = zeros(1, length(t));

for ii = 1:length(t)
    
    tOverr = t(ii)/r(ii);
    
    if (tOverr <= 2) && (tOverr > .1)
        
        C1Bending(ii) = .947 + (1.206 * sqrt(tOverr)) - (.13 * tOverr);
        C2Bending(ii) = .022 - (3.405 * sqrt(tOverr)) + (.915 * tOverr);
        C3Bending(ii) = .869 + (1.777 * sqrt(tOverr)) - (.555 * tOverr);
        C4Bending(ii) = -.81 + (.422 * sqrt(tOverr)) - (.256* tOverr);
        
    elseif (tOverr <= 20) && (tOverr > 2)
        
        C1Bending(ii) = 1.232 + (.832 * sqrt(tOverr)) - (.008 * tOverr);
        C2Bending(ii) = -3.813 + (.968 * sqrt(tOverr)) - (.26 * tOverr);
        C3Bending(ii) = 7.423 - (4.868 * sqrt(tOverr)) + (.869 * tOverr);
        C4Bending(ii) = -3.839 + (3.07 * sqrt(tOverr)) - (.6* tOverr);
        
    end
    
end

% Initializing Kt vecotr
Kt = zeros(1, length(D));
   
% Solving for Kt
for ii = 1:length(D)
    
Kt(ii) = C1Bending(ii) + (C2Bending(ii) * (2 * t(ii) /D(ii))) + ...
         (C3Bending(ii) * ((2 * t(ii) / D(ii))^2)) + ...
         (C4Bending(ii) * ((2 * t(ii) / D(ii))^3));
     
end

% Replaicing Kt for the keway (entry 1,3)
Kt(3) = 2.13;
% Replaicing Kt for the snapringt (entry 1,4)
Kt(4) = 5;

% Notch factors

% Initializing not factor vecot
q = zeros(1, length(d));

% Solving for notch factor
for ii = 1:length(q)
    
    q(ii) = 1 / (1 + (neuberBending / sqrt(r(ii))));
    
end

% Reduced stress concentration factor

% Initializing Kf vecotor
Kf = zeros(1, length(d));

for ii = 1:length(d)
    
    Kf(ii) = 1 + (q(ii) * (Kt(ii) - 1));
    
end

% ----- Reduced Stress Concentration Factor for Torsion -----

% Neuber Constnat Torison (sqrt(a))
neuberTorsion = .246 - (3.08e-3 * Sut) + (1.51e-5 * (Sut^2)) - ...
                (2.67e-8 * (Sut^3));
            
% Geometric stress-concentration factor (Kt)

% Initializing C factor vectors
C1Torsion = zeros(1, length(t));
C2Torsion = zeros(1, length(t));
C3Torsion = zeros(1, length(t));
C4Torsion = zeros(1, length(t));

for ii = 1:length(t)
    
    tOverr = t(ii)/r(ii);
    
    if (tOverr <= 10) && (tOverr >= .25)
        
        C1Torsion(ii) = .905 + (.783 * sqrt(tOverr)) - (.075 * tOverr);
        C2Torsion(ii) = -.437 - (1.969 * sqrt(tOverr)) + (.553 * tOverr);
        C3Torsion(ii) = 1.557 + (1.703 * sqrt(tOverr)) - (.578 * tOverr);
        C4Torsion(ii) = -1.061 + (.171 * sqrt(tOverr)) + (.086 * tOverr);
        
    else
        
        fprintf('Invalid Dimensions');
        
    end
end

% Initializing Kt vecotr
Kts = zeros(1, length(D));
   
% Solving for Kt
for ii = 1:length(D)
    
Kts(ii) = C1Torsion(ii) + (C2Torsion(ii) * (2 * t(ii) /D(ii))) + ...
         (C3Torsion(ii) * ((2 * t(ii) / D(ii))^2)) + ...
         (C4Torsion(ii) * ((2 * t(ii) / D(ii))^3));
     
end

% Replaicing Kt for the keway (entry 1,3)
Kts(3) = 3;
% Replaicing Kt for the snap ring (entry 1,4)
Kts(4) = 3;

% Notch factors

% Initializing not factor vecot
qs = zeros(1, length(d));

% Solving for notch factor
for ii = 1:length(qs)
    
    qs(ii) = 1 / (1 + (neuberTorsion / sqrt(r(ii))));
    
end

% Reduced stress concentration factor

% Initializing Kf vecotor
Kfs = zeros(1, length(d));

for ii = 1:length(d)
    
    Kfs(ii) = 1 + (qs(ii) * (Kts(ii) - 1));
    
end

%Calculating moments and torques at critical colations

% Initialiaing moment vector
M = zeros(1, length(xCritical));

for ii = 1:length(xCritical)
    
    if xCritical(ii) > loadPosition
        
        M(ii) = sqrt(((-reactionForces(4,1) * xCritical(ii)) + b)^2 ...
            + ((-reactionForces(3,1) * xCritical(ii)) + b2)^2);
    else
        
        
        M(ii) = sqrt((reactionForces(2, 1) * xCritical(ii))^2 + ...
                    (-reactionForces(1,1) * xCritical(ii))^2);
        
    end
end

% Torque on shaft
T = [0 1350 1350 1350 1350];

% Initialising FS vector
n = zeros(1, length(d));

for ii = 1:length(d)
    
    
    n(ii) = (16 / (pi * (d(ii) ^ 3))) * ...
            ((((4 * ((Kf(ii) * M(ii) / Se(ii))^2))) + ...
            ((3 * (Kfs(ii) * (T(ii) / Sy) ^ 2)))) ^ -.5);
    
end


