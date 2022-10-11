%% Component Dimensions and Specifications

% ----- Oil Pump -----

% Bolt Circle [in]
accessories(1).dimensions.boltCircle = {2.97 'in'};

% D-flange [in]
accessories(1).dimensions.dFlange = {3.47 'in'};

% D-Body [in]
accessories(1).dimensions.dBody = {2.55 'in'};

% L-Body [in]
accessories(1).dimensions.lBody = {4.075 'in'};

% L-CG [in]
accessories(1).dimensions.lCG = {3.78 'in'};

% t-Flange [in]
accessories(1).dimensions.tFlange = {.115 'in'};

% Drive Speed & Tolerance [rpm]
accessories(1).specs.driveSpeed = {8500 'rpm'};
accessories(1).specs.driveSpeelTol = {10 ''}; % Bilateral

% Mating Drive [# of Spines]
accessories(1).specs.matingDrive = {8 'spines'};

% Number of Bolts
accessories(1).specs.numBolts = {6 'bolts'};

% ----- Fuel Pump -----

% Bolt Circle [in]
accessories(2).dimensions.boltCircle = {3.68 'in'};

% D-flange [in]
accessories(2).dimensions.dFlange = {4.11 'in'};

% D-Body [in]
accessories(2).dimensions.dBody = {3.26 'in'};

% L-Body [in]
accessories(2).dimensions.lBody = {4.575 'in'};

% L-CG [in]
accessories(2).dimensions.lCG = {2.48 'in'};

% t-Flange [in]
accessories(2).dimensions.tFlange = {.115 'in'};

% Drive Speed & Tolerance [rpm]
accessories(2).specs.driveSpeed = {13950 'rpm'};
accessories(2).specs.driveSpeelTol = {10 ''}; % Bilateral

% Mating Drive [# of Spines]
accessories(2).specs.matingDrive = {8 'spines'};

% Number of Bolts
accessories(2).specs.numBolts = {6 'bolts'};

% ----- Hydraulic Pump -----

% Bolt Circle [in]
accessories(3).dimensions.boltCircle = {4.25 'in'};

% D-flange [in]
accessories(3).dimensions.dFlange = {4.6 'in'};

% D-Body [in]
accessories(3).dimensions.dBody = {3.75 'in'};

% L-Body [in]
accessories(3).dimensions.lBody = {5.855 'in'};

% L-CG [in]
accessories(3).dimensions.lCG = {4.46 'in'};

% t-Flange [in]
accessories(3).dimensions.tFlange = {.125 'in'};

% Drive Speed & Tolerance [rpm]
accessories(3).specs.driveSpeed = {3500 'rpm'};
accessories(3).specs.driveSpeelTol = {10 ''}; % Bilateral

% Mating Drive [# of Spines]
accessories(3).specs.matingDrive = {12 'spines'};

% Number of Bolts
accessories(3).specs.numBolts = {12 'bolts'};

% ----- Electrical Generator -----

% Bolt Circle [in]
accessories(4).dimensions.boltCircle = {4.25 'in'};

% D-flange [in]
accessories(4).dimensions.dFlange = {4.6 'in'};

% D-Body [in]
accessories(4).dimensions.dBody = {3.75 'in'};

% L-Body [in]
accessories(4).dimensions.lBody = {5.855 'in'};

% L-CG [in]
accessories(4).dimensions.lCG = {4.13 'in'};

% t-Flange [in]
accessories(4).dimensions.tFlange = {.125 'in'};

% Drive Speed & Tolerance [rpm]
accessories(4).specs.driveSpeed = {6795 'rpm'};
accessories(4).specs.driveSpeelTol = {10 ''}; % Bilateral

% Mating Drive [# of Spines]
accessories(4).specs.matingDrive = {12 'spines'};

% Number of Bolts
accessories(4).specs.numBolts = {12 'bolts'};


%% Load Calculations

for ii = 1:length(accessories)
    
    % Forces due to weight
    accessories(ii).loads.dueToWeight{1} = ...
        -(accessories(ii).specs.weight{1}(3)) ./ ...
        accessories(ii).specs.numBolts{1};
    
    % Adding units to cell array
    accessories(ii).loads.dueToWeight{1, 2} = {'lbf'};
    
    % Forces due to torsion (Magnitude)
    % F = Tmax / numBolts * boltCircleRadius
    accessories(ii).loads.dueToTorsion{1} = ...
        accessories(ii).specs.maxTorque{1} ./  ...
        (accessories(ii).specs.numBolts{1} .* ...
        accessories(ii).dimensions.boltCircle{1} * .5);

    % Adding units to cell array
    accessories(ii).loads.dueToTorsion{1, 2} = {'lbf - MAGNITUDE'};
            
    % Due to bending moment (Max Tesnsile Force)
    accessories(ii).loads.dueToBending{1} = ...
        (accessories(ii).dimensions.lCG{1} .* ...
        (-2 * accessories(ii).specs.weight{1}(3)))  ./ ...
        (accessories(ii).dimensions.boltCircle{1} * .5 ...
        * accessories(ii).specs.numBolts{1});
    
    % Adding units to cell array
    accessories(ii).loads.dueToBending{2} = {'lbf - MAX'};
    
    
    % Calculations for accesorries with 6 bolt pattern
    if accessories(ii).specs.numBolts{1} == 6
        
        % Defining angles for bolt position
        % Starts w/ left most bolt and goes cw
        angles = 0:60:300;
        
        % Initializing empty tensile load vector
        accessories(ii).loads.axialLoads = zeros(1, length(angles));
        
        for jj = 1:length(angles)
            
            % ----- Finding components of forces due to torsion -----
            % Calculating x-components
            accessories(ii).loads.dueToTorsionX{1}(jj) = ...
                accessories(ii).loads.dueToTorsion{1} * sind(angles(jj));

            % Adding units to cell array
            accessories(ii).loads.dueToTorsionX{1, 2} = {'lbf - x'};
            
            % Calculating y-components
            accessories(ii).loads.dueToTorsionY{1}(jj) = ...
                accessories(ii).loads.dueToTorsion{1} * cosd(angles(jj));
            
            % Adding units to cell array
            accessories(ii).loads.dueToTorsionY{2} = {'lbf - y'};
            
            % Calculating combined loading
            
            % X-component Combined loading
            accessories(ii).loads.combined{1} =  ...
                accessories(ii).loads.dueToTorsionX{1};
            
            % Adding units to cell Array
            accessories(ii).loads.combined{1, 2} = {'lbf - x'};
            
            % Y-component combined loading
            accessories(ii).loads.combined{2, 1} = ...
                accessories(ii).loads.dueToTorsionY{1} + ...
                accessories(ii).loads.dueToWeight{1};
            
            % Adding units to cell Array
            accessories(ii).loads.combined{2, 2} = {'lbf - y'};
            
            % ----- Finding Axial Load due to Bending -----
            accessories(ii).loads.axialLoads(jj) =  ...
                accessories(ii).loads.dueToBending{1} ...
                * sind(angles(jj));
            
            
        end
    % Calculations for accesorries with 12 bolt pattern
    else
         
        % Defining angles for bolt position
        % Starts w/ left most bolt and goes cw
        angles = 0:30:330;
        
        % Initializing empty tensile load vector
        accessories(ii).loads.axialLoads = zeros(1, length(angles));
        
        for jj = 1:length(angles)
            
            % ----- Finding components of forces due to torsion -----
            % Calculating x-components
            accessories(ii).loads.dueToTorsionX{1}(jj) = ...
                accessories(ii).loads.dueToTorsion{1} * sind(angles(jj));
            
            % Adding units to cell array
            accessories(ii).loads.dueToTorsionX{2} = {'lbf - x'};
            
            % Calculating y-components
            accessories(ii).loads.dueToTorsionY{1}(jj) = ...
                accessories(ii).loads.dueToTorsion{1} * cosd(angles(jj));
            
            % Adding units to cell array
            accessories(ii).loads.dueToTorsionY{1, 2} = {'lbf - y'};
            
            % Calculating combined loading
            
            % X-component Combined loading
            accessories(ii).loads.combined{1} =  ...
                accessories(ii).loads.dueToTorsionX{1};
            
            % Adding units to cell Array
            accessories(ii).loads.combined{1, 2} = {'lbf - x'};
            
            % Y-component combined loading
            accessories(ii).loads.combined{2, 1} = ...
                accessories(ii).loads.dueToTorsionY{1} + ...
                accessories(ii).loads.dueToWeight{1};
            
            % Adding units to cell Array
            accessories(ii).loads.combined{2, 2} = {'lbf - y'};
            
            % ----- Finding Axial Load due to Bending -----
            accessories(ii).loads.axialLoads(jj) =  ...
                accessories(ii).loads.dueToBending{1} ...
                * sind(angles(jj));
            
                  
        end
    end      
end
