%% Force acting on gear B

% Creating symbolic variable
syms Fb

% Sum of torques == 0
sumTorques = -160 * cosd(20) * 300 + ...
    (Fb * cosd(25) * 150) == 0;

% Solving for Fb
Fb = solve(sumTorques, Fb);

% Displaying solution
fprintf('Fb = %f kN \n', Fb);

%% Bearing Reaction Forces

% Definig symbolic variables
syms Rcz Roz Rcy Roy

% Sum of Forces in y == 0
sumForcesY = (-160 * sind(20)) - (Fb * sind(25)) + ...
    Rcy + Roy == 0;

% Sum of Forces in z == 0
sumForcesZ = (-160 * cosd(20)) + (Fb * cosd(25)) + ...
    Rcz + Roz == 0;

% Sum of Moments about y-axis
sumMomentsY = (160 * cosd(20) * 400) -  ...
              (Fb * cosd(25) * 750) - ...
              (Rcz * 1050) == 0;
          
% Sum of Moments about z-axis
sumMomentsZ = (-160 * sind(20) * 400) - ...
              (Fb * sind(25) * 750) + ...
              (Rcy * 1050) == 0;
          
% Solving system of equations

[A, b] = equationsToMatrix([sumForcesY ...
                            sumForcesZ ...
                            sumMomentsY ...
                            sumMomentsZ], ...
                            [Rcy Roy Rcz Roz]);
                        
bearingReactions = linsolve(A, b);

% Displaying solutions
fprintf('Rcy = %f kN \n', bearingReactions(1, 1));
fprintf('Roy = %f kN \n', bearingReactions(2, 1));
fprintf('Rcz = %f kN \n', bearingReactions(3, 1));
fprintf('Roz = %f kN \n', bearingReactions(4, 1));

%% Max Stresses

Tmax = 160 * cosd(20) * 300;
Mmax = sqrt((300 * bearingReactions(1,1))^2 + ...
            (300 * bearingReactions(3,1))^2 );

% Max stress due to bending
sigmaMax = (Mmax * 32 / (pi * (50^3))) * 10^3;

% Displaying solution
fprintf('Bending Stress @ Point of Max Bending = %f MPa \n', ...
    sigmaMax);

% Max Stress due to torsion
tauXY = (16 * Tmax / (pi * (50^3))) * 10^3;

% Displaying Solution
fprintf('Torsional Shear @ Point of Max Bending = %f MPa \n', ...
    tauXY);

%% Principal Stresses

sigma1 = (sigmaMax / 2) + sqrt((sigmaMax / 2)^2 + tauXY^2);
sigma2 = (sigmaMax / 2) - sqrt((sigmaMax / 2)^2 + tauXY^2);
maxShear = sqrt((sigmaMax / 2)^2 + (tauXY^2));

fprintf('Sigma 1 = %f MPa \n', sigma1);
fprintf('Sigma 2 = %f MPa \n', sigma2);
fprintf('Max Shear = %f MPa \n', maxShear);











