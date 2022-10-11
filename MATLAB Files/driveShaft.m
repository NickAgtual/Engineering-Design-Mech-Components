clear; close all; clc

%% Bevel gear force analysis

% Pitch angel (deg)
bevel.phi = 20;

% Number of teeth on mating beveled gear
pinion.N = 28;

% Number of teeth on beveled gear
bevel.N = 40;

% Torque on beveled gear (lbf * in)
bevel.T = 964.7;

% Pitch diameter of beveled gear (in)
bevel.d = 5;

% Tangential force (lbf)
bevel.Wt = -bevel.T / (bevel.d / 2);

% Pitch angel of pinion (deg)
pinion.gamma = atand(pinion.N / bevel.N);

% Radial Force (lbf)
bevel.Wr = bevel.Wt * tand(bevel.phi) * cosd(pinion.gamma);

% Axial Force (lbf)
bevel.Wa = bevel.Wt * tand(bevel.phi) * sind(pinion.gamma);

% Bevel face width
bevel.faceWidth = 1;

% Bearing width 
bearing.width = .375;
bearing.diameter = .375;

%% Drive Gear (Gear 2 - Right Side (E-gen & F-pump) Force Analysis

gear2R.Wt = 229.69;
gear2R.angle = 346.69;
gear2R.faceWidth = 1;

% Definign symbolic variables
syms RxGear2R RyGear2R

% Sum of forces for gear2R
sumForcesX2R = (-gear2R.Wt * sind(gear2R.angle)) - ...
               (gear2R.Wt * tand(bevel.phi) * cosd(gear2R.angle)) + ...
                RxGear2R == 0;
           
sumForcesY2R = (gear2R.Wt * cosd(gear2R.angle)) - ...
               (gear2R.Wt * tand(bevel.phi) * sind(gear2R.angle)) + ...
                RyGear2R == 0;

% Solving equillibrium equations
RxGear2R = solve(sumForcesX2R, RxGear2R);
RyGear2R = solve(sumForcesY2R, RyGear2R);

%% Drive Gear (Gear 2 - Left Side (H-pump & O-pump) Force Analysis

gear2L.Wt = 438.5;
gear2L.angle = 14.94;
gear2L.faceWidth = .75;

% Definign symbolic variables
syms RxGear2L RyGear2L

% Sum of forces for gear2R
sumForcesX2L = (-gear2L.Wt * sind(gear2L.angle)) - ...
               (gear2L.Wt * tand(bevel.phi) * cosd(gear2L.angle)) + ...
                RxGear2L == 0;
            
sumForcesY2L = (-gear2L.Wt * cosd(gear2L.angle)) - ...
               (gear2L.Wt * tand(bevel.phi) * sind(gear2L.angle)) + ...
                RyGear2L == 0;

% Solving equillibrium equations
RxGear2L = solve(sumForcesX2L, RxGear2L);
RyGear2L = solve(sumForcesY2L, RyGear2L);

%% Equillibrium Equations for the Shaft

% Defining symbolic variables
syms RAx RAy RAz RBx RBy RBz

shaft.Length = 6.595;
shaft.LengthBearingToBearing = shaft.Length - bearing.width;

bevel.loadPosition = 4.72;
gear2R.loadPosition = 2.4375; % Relative to center of bearing
gear2L.loadPosition = 1.0625; % Relative to center of bearing

% Construction force and position vectors
% Assuming point A to be the origin
% [i j k]

reactionA.force = [RAx RAy RAz];
reactionA.position = [0 0 0];

reactionB.force = [RBx RBy 0];
reactionB.position = [0 0 shaft.LengthBearingToBearing];

gear2R.force = [RxGear2R RyGear2R 0];
gear2R.position = [0 0 gear2R.loadPosition];

gear2L.force = [RxGear2L RyGear2L 0];
gear2L.position = [0 0 gear2L.loadPosition];

bevel.force = [bevel.Wr bevel.Wt bevel.Wa];
bevel.position = [0 0 bevel.loadPosition];

% Sum of forces in x and y directions
% DETERMINE DIRECTION OF BEVEL COMPONENTS
sumForcesX = RAx + RBx + RxGear2L + RxGear2R + bevel.Wr == 0; 
sumForcesY = RAy + RBy + RyGear2L + RyGear2R + bevel.Wt == 0; 
% Assuming only one bearing supports axial load
sumForcesZ = RAz + bevel.Wa == 0; 

% Sum of moments
sumMoments = cross(reactionA.position, reactionA.force) + ...
             cross(reactionB.position, reactionB.force) + ...
             cross(gear2R.position, gear2R.force) + ...
             cross(gear2L.position, gear2L.force) + ...
             cross(bevel.position, bevel.force) == 0;
         
% Setting up matrices for system of linear equations

% Equations
eqns = [sumForcesX sumForcesY sumForcesZ sumMoments];

% Unknonw variables
unknownVals = [RAx RAy RAz RBx RBy];

% Creating matrices
[A, b] = equationsToMatrix(eqns, unknownVals);

% Solving system of equations
shaftReactionForces = linsolve(A, b);

%% Shear and Bending Moment Diagrams

% ----- y-z plane -----

% Creating new figure
figure(1)

% Shear diagram
subplot(2, 2, 1)

syms x

shearYZ = piecewise(x < 0, 0, ...
          0 < x < gear2L.loadPosition, shaftReactionForces(2,1), ...
          gear2L.loadPosition < x < gear2R.loadPosition, ...
          shaftReactionForces(2,1) + RyGear2L, ...
          gear2R.loadPosition < x < bevel.loadPosition, ...
          shaftReactionForces(2,1) + RyGear2L + RyGear2R, ...
          bevel.loadPosition < x < shaft.LengthBearingToBearing, ...
          shaftReactionForces(2,1) + RyGear2L + RyGear2R + ...
          bevel.Wt, x > shaft.LengthBearingToBearing, 0);

% Plotting shear  diagram 
fplot(shearYZ);

% Plot parameters
hold on
grid on
grid minor

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, x > shaft.LengthBearingToBearing, 0);

% Plotting while lines
whitePlot = fplot(white, 'Color', [1 1 1]);

% Turning asymptotes off
whitePlot.ShowPoles = 'Off';

% Plotting y = 0
yline(0, 'k')

% Setting axis limits
xlim([-.25 shaft.LengthBearingToBearing + .25]);
ylim([-400 500]);

% Axis Descriptors
xlabel('\emph {z - distance (in)}', ...
    'fontsize', 12, 'Interpreter', 'latex');
ylabel('\emph {V (lbf)}', 'fontsize', 12, 'Interpreter', 'latex');
title('\emph {Shear Diagram (y-z Plane)}', 'fontsize', ...
14, 'Interpreter', 'latex');

% Moment diagram
subplot(2, 2, 2)

syms b1 b2 b3

% Equation solving for y-intercept 1
interceptEq1YZ = (shaftReactionForces(2, 1) * gear2L.loadPosition) == ...
    ((shaftReactionForces(2,1) + RyGear2L) * gear2L.loadPosition) + b1;

% solving for y-intercept 1
b1 = solve(interceptEq1YZ, b1);

% Equation for y-intercept 2
interceptEq2YZ = ((shaftReactionForces(2, 1) * gear2L.loadPosition)) + ...
    ((shaftReactionForces(2, 1) + RyGear2L) *  ...
    (gear2R.loadPosition - gear2L.loadPosition)) == ...
    (shaftReactionForces(2,1) + RyGear2L + RyGear2R) * ...
    gear2R.loadPosition+ b2;

% solving for y-intercept 2
b2 = solve(interceptEq2YZ, b2);

% Equation for y-intercept 3
interceptEq3YZ = ((shaftReactionForces(2, 1) * gear2L.loadPosition)) + ...
    ((shaftReactionForces(2, 1) + RyGear2L) *  ...
    (gear2R.loadPosition - gear2L.loadPosition)) + ...
    ((shaftReactionForces(2,1) + RyGear2L + RyGear2R) * ...
    (bevel.loadPosition - gear2R.loadPosition)) == ...
    (shaftReactionForces(2,1) + RyGear2L + RyGear2R + bevel.Wt) * ...
    bevel.loadPosition+ b3;

% solving for y-intercept 3
b3 = solve(interceptEq3YZ, b3);

% Piecewsie moment equations
momentYZEq1 = shaftReactionForces(2, 1) * x;
momentYZEq2 = b1 + ((shaftReactionForces(2, 1) + RyGear2L)*x);
momentYZEq3 = b2 + ((shaftReactionForces(2,1) + RyGear2L + RyGear2R) *x);
momentYZEq4 = b3 + ((shaftReactionForces(2,1) + RyGear2L + RyGear2R + ...
    bevel.Wt ) *x);

% Combined piecewise equation
momentYZ = piecewise(x < 0, 0, 0 < x < gear2L.loadPosition, ...
           momentYZEq1, gear2L.loadPosition < x < gear2R.loadPosition, ...
           momentYZEq2, gear2R.loadPosition < x < bevel.loadPosition, ...
           momentYZEq3, bevel.loadPosition < x < ...
           shaft.LengthBearingToBearing, momentYZEq4, x > ...
           shaft.LengthBearingToBearing, 0);
       
% Plotting moment diagram
fplot(momentYZ, 'r')

% Setting axis limits
xlim([-.25 shaft.LengthBearingToBearing + .25]);
ylim([-400 500]);

% Plot parameters
hold on
grid on
grid minor

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, x > shaft.LengthBearingToBearing, 0);

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

% ----- x-z plane -----

% Shear diagram
subplot(2, 2, 3)

shearXZ = piecewise(x < 0, 0, ...
          0 < x < gear2L.loadPosition, shaftReactionForces(1,1), ...
          gear2L.loadPosition < x < gear2R.loadPosition, ...
          shaftReactionForces(1,1) + RxGear2L, ...
          gear2R.loadPosition < x < bevel.loadPosition, ...
          shaftReactionForces(1,1) + RxGear2L + RxGear2R, ...
          bevel.loadPosition < x < shaft.LengthBearingToBearing, ...
          shaftReactionForces(1,1) + RxGear2L + RxGear2R + ...
          bevel.Wr, x > shaft.LengthBearingToBearing, 0);

% Plotting shear  diagram 
fplot(shearXZ);

% Plot parameters
hold on
grid on
grid minor

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, x > shaft.LengthBearingToBearing, 0);

% Plotting while lines
whitePlot = fplot(white, 'Color', [1 1 1]);

% Turning asymptotes off
whitePlot.ShowPoles = 'Off';

% Plotting y = 0
yline(0, 'k')

% Setting axis limits
xlim([-.25, shaft.LengthBearingToBearing + .25]);
ylim([-300 150]);

% Axis Descriptors
xlabel('\emph {z - distance (in)}', ...
    'fontsize', 12, 'Interpreter', 'latex');
ylabel('\emph {V (lbf)}', 'fontsize', 12, 'Interpreter', 'latex');
title('\emph {Shear Diagram (x-z Plane)}', 'fontsize', ...
14, 'Interpreter', 'latex');

% Moment diagram
subplot(2, 2, 4)

syms b4 b5 b6

% Equation solving for y-intercept 4
interceptEq1XZ = (shaftReactionForces(1, 1) * gear2L.loadPosition) == ...
    ((shaftReactionForces(1,1) + RxGear2L) * gear2L.loadPosition) + b4;

% solving for y-intercept 4
b4 = solve(interceptEq1XZ, b4);

% Equation for y-intercept 5
interceptEq2XZ = ((shaftReactionForces(1, 1) * gear2L.loadPosition)) + ...
    ((shaftReactionForces(1, 1) + RxGear2L) *  ...
    (gear2R.loadPosition - gear2L.loadPosition)) == ...
    (shaftReactionForces(1,1) + RxGear2L + RxGear2R) * ...
    gear2R.loadPosition + b5;

% solving for y-intercept 5
b5 = solve(interceptEq2XZ, b5);

% Equation for y-intercept 6
interceptEq3XZ = ((shaftReactionForces(1, 1) * gear2L.loadPosition)) + ...
    ((shaftReactionForces(1, 1) + RxGear2L) *  ...
    (gear2R.loadPosition - gear2L.loadPosition)) + ...
    ((shaftReactionForces(1,1) + RxGear2L + RxGear2R) * ...
    (bevel.loadPosition - gear2R.loadPosition)) == ...
    (shaftReactionForces(1,1) + RxGear2L + RxGear2R + bevel.Wr) * ...
    bevel.loadPosition + b6;

% solving for y-intercept 6
b6 = solve(interceptEq3XZ, b6);

% Piecewsie moment equations
momentXZEq1 = shaftReactionForces(1, 1) * x;
momentXZEq2 = b4 + ((shaftReactionForces(1,1) + RxGear2L) * x);
momentXZEq3 = b5 + ((shaftReactionForces(1,1) + RxGear2L + RxGear2R) * x);
momentXZEq4 = b6 + ((shaftReactionForces(1,1) + RxGear2L + RxGear2R + ...
                    bevel.Wr) * x);

% Combined piecewise equation
momentXZ = piecewise(x < 0, 0, 0 < x < gear2L.loadPosition, ...
           momentXZEq1, gear2L.loadPosition < x < gear2R.loadPosition, ...
           momentXZEq2, gear2R.loadPosition < x < bevel.loadPosition, ...
           momentXZEq3, bevel.loadPosition < x < shaft.LengthBearingToBearing, ...
           momentXZEq4, x > shaft.LengthBearingToBearing, 0);
       
% Plotting moment diagram
fplot(momentXZ, 'r')

% Setting axis limits
xlim([-.25 shaft.LengthBearingToBearing + .25]);
ylim([-300 150]);

% Plot parameters
hold on
grid on
grid minor

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, x > shaft.LengthBearingToBearing, 0);

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
sgtitle('Accessory Drive Shaft', 'fontsize', 16, 'Interpreter', ...
    'latex');

%% Shaft Stress Analysis

% Shaft diameter
d = [bearing.diameter .45 .45 .45 .45 .5 .55 .55 .45 .45 .45 ...
    bearing.diameter];
% Elements 2,3,4,5,6,10,11 are for the keway and snap ring and will be
%replaced

% Neighboring diameter
D = [.45 .45 .45 .45 .45 .45 .6 .6 .55 .45 .45 .45];
% Elements 2,3,4,5,6,10,11 are for the keway and snap ring and will be
%replaced

% Critical loactions
xCritical = [bearing.width/2 (gear2L.loadPosition - (gear2L.faceWidth/2)) ...
            gear2L.loadPosition (gear2L.loadPosition + (gear2L.faceWidth/2)) ...
            (gear2R.loadPosition - (gear2R.faceWidth/2)) gear2R.loadPosition ...
            (gear2R.loadPosition + (gear2R.faceWidth/2)) 3.485 ...
            (bevel.loadPosition - (bevel.faceWidth/2)) bevel.loadPosition ...
            (bevel.loadPosition + (bevel.faceWidth/2)) ...
            (shaft.LengthBearingToBearing - (bearing.width/2))];

% All factors use the following format if necessary:
% factor = [bendingFactor torsionFactor]

%  4142 Q&T 600F  Steel 
Sut = 280; % Ulitmate strength kpsi 280
Sy = 250; % Yield strength kpsi 250

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

r = ones(1, length(d)) * .15;

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

% Replaicing Kt for the snap ring [Gear 2L] (entry 1,2)
Kt(2) = 5;
% Replaicing Kt for the key way [Gear 2L] (entry 1,3)
Kt(3) = 2.13;

% Replaicing Kt for the snap ring [Gear 2R] (entry 1,5)
Kt(5) = 5;
% Replaicing Kt for the key way [Gear 2R] (entry 1,6)
Kt(6) = 2.13;

% Replaicing Kt for the snap ring [Bevel Gear] (entry 1,11)
Kt(11) = 5;
% Replaicing Kt for the key way [Bevel Gear] (entry 1,10)
Kt(10) = 2.13;


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
neuberTorsion = .19 - (2.5e-3 * Sut) + (1.35e-5 * (Sut^2)) - ...
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
        
        fprintf('Invalid Dimensions \n');
        
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

% Replaicing Kts for the snap ring [Gear 2L] (entry 1,2)
Kts(2) = 3;
% Replaicing Kts for the key way [Gear 2L] (entry 1,3)
Kts(3) = 3;

% Replaicing Kts for the snap ring [Gear 2R] (entry 1,5)
Kts(5) = 3;
% Replaicing Kts for the key way [Gear 2R] (entry 1,6)
Kts(6) = 3;

% Replaicing Kts for the snap ring [Bevel Gear] (entry 1,11)
Kts(11) = 3;
% Replaicing Kts for the key way [Bevel Gear] (entry 1,10)
Kts(10) = 3;

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
    
    if xCritical(ii) <= gear2L.loadPosition
        
        M(ii) = sqrt(((shaftReactionForces(2, 1) * xCritical(ii))^2) + ...
                ((shaftReactionForces(1, 1) * xCritical(ii))^2));
             
    elseif xCritical(ii) > gear2L.loadPosition && ...
            xCritical(ii) <= gear2R.loadPosition
        
        M(ii) = sqrt((b1 + ((shaftReactionForces(2, 1) + RyGear2L)* ...
            xCritical(ii))^2) + ((b4 + ((shaftReactionForces(1,1) + ...
            RxGear2L) * xCritical(ii)))^2));
        
    elseif xCritical(ii) > gear2R.loadPosition && ...
            xCritical(ii) <= bevel.loadPosition
        
         M(ii) = sqrt(((b2 + ((shaftReactionForces(2,1) + RyGear2L + ...
             RyGear2R) * xCritical(ii)))^2) + ((b5 + ...
             ((shaftReactionForces(1,1) + RxGear2L + RxGear2R) * ...
             xCritical(ii)))^2));
         
    else
        
        M(ii) = sqrt(((b3 + ((shaftReactionForces(2,1) + RyGear2L + ...
            RyGear2R + bevel.Wt ) * xCritical(ii)))^2) + ...
            ((b6 + ((shaftReactionForces(1,1) + RxGear2L + RxGear2R + ...
                    bevel.Wr) * xCritical(ii)))^2));
        
    end
end

% Torque on shaft @ different critical locations
T = [0 0 0 (964.7 - 482.35) (964.7 - 482.35) (964.7 - 482.35) 964.7 ...
    964.7 964.7 964.7 0 0];

% Initialising FS vector
n = zeros(1, length(d));

for ii = 1:length(d)
    
    n(ii) = (16 / (pi * (d(ii) ^ 3))) * ...
            ((((4 * ((Kf(ii) * M(ii) / Se(ii))^2))) + ...
            ((3 * (Kfs(ii) * (T(ii) / Sy) ^ 2)))) ^ -.5);
    
end

%% Torque Diagram

figure(2)

torquePiecewise = piecewise(x < gear2L.loadPosition, 0, ...
                   gear2L.loadPosition < x < gear2R.loadPosition,...
                   (964.7/2), gear2R.loadPosition < x < ...
                   bevel.loadPosition, 964.7, x > bevel.loadPosition, 0);
% Plotting y = 0
yline(0, 'k--')

% Plot parameters
hold on
grid on
grid minor

fplot(torquePiecewise, 'Color', [.466 .674 .188]);

% Setting axis limits
xlim([-.25, shaft.LengthBearingToBearing + .25]);
ylim([-50 1000]);

% Piecewise function for appearance (no line below 0 and greater than
% length)
white = piecewise(x < 0, 0, ...
    x > shaft.LengthBearingToBearing, 0);

% Plotting while lines
whitePlot = fplot(white, 'Color', [1 1 1]);

% Turning asymptotes off
whitePlot.ShowPoles = 'Off';

% Axis Descriptors
xlabel('\emph {z - distance (in)}', ...
    'fontsize', 14, 'Interpreter', 'latex');
ylabel('\emph {T (lbf$\cdot$in)}', 'fontsize', 14, 'Interpreter', 'latex');
title('\emph {Torque Diagram - Accessory Drive Shaft}', 'fontsize', ...
16, 'Interpreter', 'latex');

%% S-N Curve

% Creating new figure
figure(3)

xSN = [10^0 10^3 10^6 10^10];
ySN = [Sut (Sut * .77) min(Se) min(Se)];

semilogx(xSN, ySN, 'k')

grid on

% Axis descriptors
xlabel('\emph {Number of Cycles}', 'fontsize', 14, 'Interpreter', 'latex');
ylabel('\emph { Strength (kpsi)}', 'fontsize', 14, 'Interpreter', 'latex');
title('\emph {S-N Curve}', 'fontsize', 16, 'Interpreter', 'latex');
