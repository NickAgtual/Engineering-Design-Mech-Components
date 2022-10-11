clear; clc; close all

% Defining accessories structure
accessories.name = 'Oil Pump';
accessories(2).name = 'Fuel Pump';
accessories(3).name = 'Hydraulic Pump';
accessories(4).name = 'Electrical Generator';
%% Weights [lbf]

% Vecor form [i, j, k]
% +i = forward
% -i = aft
% +j = outboard
% -j = inboard
% +k = up
% -k = down

% Gearbox housing and gear train components w/o accessories
gearbox.weight = {[0 0 -423] 'lbf'};

% Oil Pump
accessories(1).specs.weight = {[0 0 -27] 'lbf'};

% Fuel Pump
accessories(2).specs.weight = {[0 0 -28] 'lbf'};

% Hydraulic Pump
accessories(3).specs.weight = {[0 0 -42] 'lbf'};

% Electrical Generator
accessories(4).specs.weight = {[0 0 -55] 'lbf'};

% Reaction at geabox thurst support (fwd-aft & inboard-outboard)
syms tSx tSy
thrustSupport.rxn = {[tSx tSy 0] 'lbf'};

% Reaction at geabox lateral support (vertical & fwd-aft)
syms lSx lSz
lateralSupport.rxn = {[lSx 0 lSz] 'lbf'};

% Reaction at gearbox aft support (inboard-outboard & vertical)
syms aSy aSz
aftSupport.rxn = {[0 aSy aSz] 'lbf'};

%% Max Torques On Accessory Components

% Oil Pump
% Max Torque [in*lbf]
% ccw = (+) lookint aft
% cw = (-) lookint aft
accessories(1).specs.maxTorque = {-74.1 'in*lbf'};

% Fuel Pump
% Max Torque [in*lbf]
% ccw = (+) lookint aft
% cw = (-) lookint aft
accessories(2).specs.maxTorque = {45.2 'in*lbf'};

% Hydraulic Pump
% Max Torque [in*lbf]
% ccw = (+) lookint aft
% cw = (-) lookint aft
accessories(3).specs.maxTorque = {602.9 'in*lbf'};

% Electrical Generator
% Max Torque [in*lbf]
% ccw = (+) lookint aft
% cw = (-) lookint aft
accessories(4).specs.maxTorque = {-1170.5 'in*lbf'};

%% Position of Loads [in]

% Reference dadatum:
% i = gearbox thrust support gearbox lateral support 
% j = gearbox thrust support (by oil pump)
% z = gearbox thrust support
% Thrust Support Position = [0 0 0]

% Vecor form [i, j, k]
% +i = forward
% -i = aft
% +j = outboard (left wing)
% -j = inboard (right wing)
% +k = up
% -k = down

% Gearbox housing w/o gear train components or accessories
gearbox.loadCoord = {[-(3.45 - 1.91), 12.22, -6.4] 'in'};

% Oil Pump
accessories(1).loadCoord = {[-(3.78 + 1.91), ...
                           (12.22 - 9.49), ...
                           -(6.4 - 3.54)] 'in'};

% Fuel Pump
accessories(2).loadCoord = {[-(2.48 + 1.91), ...
                           (12.22 + 10.05), ...
                           -(6.4 - 6.02)] 'in'};

% Hydraulic Pump
accessories(3).loadCoord = {[-(4.46 + 1.91), ...
                            (12.22 - 5.73), ...
                            -(6.4 - 1.42)] 'in'};

% Electrical Generator
accessories(4).loadCoord = {[-(4.13 + 1.91), ...
                            (12.22 + 6.3), ...
                            -(6.4 - 2.27)] 'in'};

% Geabox Thurst Support [Datum] (by oil pump)
thrustSupport.loadCoord = {zeros(1, 3) 'in'};

% Geabox Lateral Support (by fuel pump)
% Z should be positive
lateralSupport.loadCoord = {[0 25.48 (9.07 - 6.4)] 'in'};

% Gearbox Aft Support 
aftSupport.loadCoord = {[-(6.55 + 1.91), ...
                        (12.22 + 4.05), ...
                        -(6.4 - 3.47)] 'in'};

%% Concentrated Moments

% Uses standard sign convention (Right hand rule)
% ccw when looking down axis is (+)

% Torque due to rotating shafts [in*lbf]
% 964.7 = accesory drive shaft
% 675.3 = compressor transfer shaft
concentratedMoments = {[0 0 674.7] 'lbf*in'};

%% Sum of Forces

% x-Direction
sumForcesX = gearbox.weight{1}(1) + ...
             accessories(1).specs.weight{1}(1) + ...
             accessories(2).specs.weight{1}(1) + ...
             accessories(3).specs.weight{1}(1) + ...
             accessories(4).specs.weight{1}(1) + ...
             thrustSupport.rxn{1}(1) + ...
             lateralSupport.rxn{1}(1) + ...
             aftSupport.rxn{1}(1) == 0;

%y-Direction
sumForcesY = gearbox.weight{1}(2) + ...
             accessories(1).specs.weight{1}(2) + ...
             accessories(2).specs.weight{1}(2) + ...
             accessories(3).specs.weight{1}(2) + ...
             accessories(4).specs.weight{1}(2) + ...
             thrustSupport.rxn{1}(2) + ...
             lateralSupport.rxn{1}(2) + ...
             aftSupport.rxn{1}(2) == 0;

% z-Direction
sumForcesZ = gearbox.weight{1}(3) + ...
             accessories(1).specs.weight{1}(3) + ...
             accessories(2).specs.weight{1}(3) + ...
             accessories(3).specs.weight{1}(3) + ...
             accessories(4).specs.weight{1}(3) + ...
             thrustSupport.rxn{1}(3) + ...
             lateralSupport.rxn{1}(3) + ...
             aftSupport.rxn{1}(3) == 0;

%% Sum of Moments About Lateral Support

% Uses standard sign convention (Right hand rule)
% ccw when looking down axis is (+)

sumMoments = cross(gearbox.loadCoord{1}, gearbox.weight{1}) + ...
             cross(accessories(1).loadCoord{1}, ...
             accessories(1).specs.weight{1}) + ...
             cross(accessories(2).loadCoord{1}, ...
             accessories(2).specs.weight{1}) + ...
             cross(accessories(3).loadCoord{1}, ...
             accessories(3).specs.weight{1}) + ...
             cross(accessories(4).loadCoord{1}, ...
             accessories(4).specs.weight{1}) + ...
             cross(thrustSupport.loadCoord{1}, ...
             thrustSupport.rxn{1}) + ...
             cross(lateralSupport.loadCoord{1}, ...
             lateralSupport.rxn{1}) + ...
             cross(aftSupport.loadCoord{1}, ...
             aftSupport.rxn{1}) + ...
             concentratedMoments{1} == 0;
 
%% Solving System of Equations

unknownVars = [tSx tSy lSx lSz aSy aSz];

[A, B] = equationsToMatrix([sumForcesX, sumForcesY, ...
                            sumForcesZ, sumMoments(1), ...
                            sumMoments(2), sumMoments(3)], ...
                            unknownVars);

rxns = linsolve(A, B);

%% Displaying Solutions

fprintf('Reaction at Thrust Support (fwd-aft-Direction)[x]:%f lbf \n', ...
    rxns(1, 1))
fprintf('Reaction at Thrust Support (inboard-outboard-Direction)[y]: %f lbf \n', ...
    rxns(2, 1))
fprintf('Reaction at Lateral Support (fwd-aft-Direction)[x]: %f lbf \n', ...
    rxns(3, 1))
fprintf('Reaction at Lateral Support (vertical-Direction)[z]: %f lbf \n', ...
    rxns(4, 1))
fprintf('Reaction at Aft Support (inboard-outboard-Direction)[y]: %f lbf \n', ...
    rxns(5, 1))
fprintf('Reaction at Aft Support (vertical-Direction)[z]: %f lbf \n', ...
    rxns(6, 1))
