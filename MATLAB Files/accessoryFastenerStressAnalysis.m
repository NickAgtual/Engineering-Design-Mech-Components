%% Defining Accessory Bolts

% Defining accessoryBolt structure and names
accessoryBolt(1).name = {'10-24-UNC-3A' ''};
accessoryBolt(2).name = {'8-32-UNC-3A' ''};

% Tensile Stress Area
accessoryBolt(1).At = {.0175 'in^2'};
accessoryBolt(2).At = {.014 'in^2'};

% Nominal Diameter
accessoryBolt(1).dNom = {.19 'in'};
accessoryBolt(2).dNom = {.1640 'in'};

%% Calculating Allowable Stress for Accessory Fasteners

% Initializing size changing components
accessories(1).loads.vecSum{1} = zeros(1, 6);
accessories(2).loads.vecSum{1} = zeros(1, 6);
accessories(3).loads.vecSum{1} = zeros(1, 6);
accessories(4).loads.vecSum{1} = zeros(1, 6);

for ii = 1:length(accessories)
    
    if accessories(ii).specs.numBolts{1} == 6
        
        for jj = 1: accessories(ii).specs.numBolts{1}
            
            % Calculating vector sum of x & y components
            accessories(ii).loads.vecSum{1}(jj) =  ...
                sqrt(accessories(ii).loads.combined{1}(jj) .^2 + ...
                accessories(ii).loads.combined{2}(jj) .^2);
            
            % Adding units to cell array
            accessories(ii).loads.vecSum{1, 2} = 'lbf';
            
            % Calculating the shear stresses on each bolt
            accessories(ii).stresses.shear(jj) = designFactorFastener * ...
                accessories(ii).loads.vecSum{1}(jj) * crashFactor / (...
                accessoryBolt(1).dNom{1}^2 * (pi/4));
            
            % Finding the maximum shear stress
            accessories(ii).stresses.maxShear =  ...
                max(accessories(ii).stresses.shear);
            
            % Calculating the normal stress on each bolt
            accessories(ii).stresses.normal(jj) = designFactorFastener  ...
                * accessories(ii).loads.axialLoads(jj) * crashFactor / ...
                (accessoryBolt(1).At{1});
            
            % Finding the max normal stress
            accessories(ii).stresses.maxNormal = ...
                max(accessories(ii).stresses.normal);
            
        end
        
    else
        
        for jj = 1: accessories(ii).specs.numBolts{1}
            
            % Calculating vector sum of x & y components
            accessories(ii).loads.vecSum{1}(jj) =  ...
                sqrt(accessories(ii).loads.combined{1}(jj) .^2 + ...
                accessories(ii).loads.combined{2}(jj) .^2);
            
            % Adding units to cell array
            accessories(ii).loads.vecSum{1, 2} = 'lbf';
            
            % Calculating the shear stresses on each bolt
            accessories(ii).stresses.shear(jj) = designFactorFastener * ...
                accessories(ii).loads.vecSum{1}(jj) * crashFactor / (...
                accessoryBolt(2).dNom{1}^2 * (pi/4));
            
            % Finding the maximum shear stress
            accessories(ii).stresses.maxShear =  ...
                max( accessories(ii).stresses.shear);
            
            % Calculating the normal stress on each bolt
            accessories(ii).stresses.normal(jj) = designFactorFastener  ...
                * accessories(ii).loads.axialLoads(jj) * crashFactor / ...
                (accessoryBolt(2).At{1});
            
            % Finding the max normal stress
            accessories(ii).stresses.maxNormal = ...
                max(accessories(ii).stresses.normal);
            
        end
    end
end

% Extracting proof strenght from excel file
proofStrength = table2array(SAEBoltStrength(:, 3))';

% Extracting tensile strenght from excel file
tensileStrength = table2array(SAEBoltStrength(:,4))';

% Initializing Arrays
% Row = accessory component
% Col = SAE bolt
shearFSstudy = zeros(length(accessories),length(proofStrength));
axialFSstudy = zeros(length(accessories),length(proofStrength)); 

% Finding FS for different bolt materials
for ii = 1: length(accessories)
    
    for jj = 1:length(proofStrength)
        
        shearFSstudy(ii, jj) = (proofStrength(jj) * 1000)  ...
            ./ accessories(ii).stresses.maxShear;
        
        axialFSstudy(ii, jj) = (tensileStrength(jj) * 1000) ./ ...
            accessories(ii).stresses.maxNormal;
        
    end
end

%% Bolt Selection
% Picking SAE Grade 1 Bolts

% Initializing Arrays
% Row = accessory component
% Col = bolt #
shearFS = zeros(length(accessories), 12);
axialFS = zeros(length(accessories),12); 

for ii = 1:length(accessories)
    
    if accessories(ii).specs.numBolts{1} == 6
        
        for jj = 1:6
            
            shearFS(ii, jj) = proofStrength(1) * 1000 ./ ...
                accessories(ii).stresses.shear(jj);
            
            axialFS(ii, jj) = tensileStrength(1) * 1000 ./ ...
                accessories(ii).stresses.normal(jj);
            
        end
    else
        
        for jj = 1:12
            
            shearFS(ii, jj) = proofStrength(1) * 1000 ./ ...
                accessories(ii).stresses.shear(jj);
            
            axialFS(ii, jj) = tensileStrength(1) * 1000 ./ ...
                accessories(ii).stresses.normal(jj);
        end
    end
end

            
        
        










