close all
%% Reading Excel Files
fileName = 'boltSpecsAndMaterialProperties.xlsx';

SAEBoltStrength = readtable(fileName, 'PreserveVariableNames', true);

aluminumAlloys = readtable(fileName,'SHEET','ALUMINUM ALLOYS', ...
    'PreserveVariableNames', true);

boltDimensions = readtable(fileName, 'SHEET', 'BOLT DIMENSIONS', ...
    'PreserveVariableNames', true);

% Must be presented in increasing inner diameter order
bearingDimensions = readtable(fileName, 'SHEET', 'BEARING DIMENSIONS', ...
    'PreserveVariableNames', true);

%% Defining Design Factor and Crash Factor

% nd
designFactorLug = 2.5;
designFactorFastener = 1.3;
% Kcrash
crashFactor = 3;

%% Determining Bolt Size

% Defining dBolt as a symbolic variable
syms DBolt

% Minimum yield strenght (kpsi) of bolts
minYieldStrengthBolt = table2array(SAEBoltStrength(1:end, 5))';

% Standard bolt sizes
stdBoltDia = table2array(boltDimensions(1:end, 2))';

% Vector sum of rection forces
% Thrust, Lateral, Aft
pLug = [sqrt(rxns(1,1)^2 + rxns(2,1)^2), ...
        sqrt(rxns(3,1)^2 + rxns(4,1)^2), ...
        sqrt(rxns(5,1)^2 + rxns(6,1)^2)];
     
% Initializing all size changing vectors
vecLength = length(minYieldStrengthBolt);
Sy = zeros(1, vecLength);
dBolt = zeros(1, vecLength, length(pLug));
dBoltStd = zeros(1, vecLength, length(pLug));
tauBolt = sym(zeros(1, length(pLug)));
     

for ii = 1:length(minYieldStrengthBolt)
         
    for loadIdx = 1:length(pLug)
             
        % Yield strength of bolt
        Sy(ii) = minYieldStrengthBolt(ii) * 1000;
             
        % Shear stress on lugbolt
        tauBolt(loadIdx) = pLug(loadIdx) * crashFactor ./ ...
            (pi * (DBolt.^2) ./ 4);
            
        % Design factor equation w/ shear stress subbed in
        materialSelection = Sy(ii) * .5 ./ tauBolt(loadIdx) ...
            == designFactorFastener;
             
        % Solving for Dbolt
        boltDiaIdx = solve(materialSelection, DBolt);
             
        % Taking positive value and storing in dBolt array
        dBolt(1, ii, loadIdx) = boltDiaIdx(1);
             
        % Determining standard bolt size
         for jj = 1: length(dBolt)
                 
             % Potential std size bolts
             potentialDia = stdBoltDia(stdBoltDia > dBolt(1, jj, loadIdx));
                 
             % Smalles potential std size bolt
             dBoltStd(1, jj, loadIdx) = min(potentialDia);
             
        end
    end
end

%% Determining Bearing Dimensions

% Extracting Bearing Inner Diameter (in)
bearingIDs = table2array(bearingDimensions(1:end, 2))';

% Extracting bearing outer diameter (in)
bearingODs = table2array(bearingDimensions(1:end, 3))';

% Extracting bearing width (in)
bearingWidths = table2array(bearingDimensions(1:end, 4))';

% Initializing all size changing vectors
bearingID = zeros(1, length(dBoltStd), length(pLug));
bearingWidth = zeros(1, length(dBoltStd), length(pLug));
bearingOD = zeros(1, length(dBoltStd), length(pLug));

for ii = 1:length(dBoltStd)
    
    for loadIdx = 1:length(pLug)
        
        % Inner diameter of bearing
        potentialBearings = bearingIDs(bearingIDs >= ...
            dBoltStd(1, ii, loadIdx));
        
        % If the bearing exists
        if ~ isempty(potentialBearings)
            
            bearingID(1, ii, loadIdx) = min(potentialBearings);
        
        % If the bearing does not exist
        else
            
            bearingID(1, ii, loadIdx) = NaN;
            
        end
        
        % Bearing width
        negativeIndex = length(potentialBearings);
        
        % If the bearing exists
        if negativeIndex > 0
            
            bearingWidth(1, ii, loadIdx) = bearingWidths(end - ...
                negativeIndex + 1);
            bearingOD(1, ii, loadIdx) = bearingODs(end - ...
                negativeIndex + 1);
        
        % If the bearing does not exist
        else
            
            bearingWidth(1, ii, loadIdx) = NaN;
            bearingWidth(1, ii, loadIDx) = NaN;
            
        end
        
    end
end

%% Determining Depth and Width of Clevis

% ----- Geometry -----
% Thickness of bushing diameter
bushingThickness = .03;

% Bushing flange thickness
bushingFlange = .031;

% Allowable gap between face of the bushing flanges and bearing
linkTol = .01;


% Initializing all size changing vectors
Dlug = zeros(1, length(dBoltStd));
Rlink = zeros(1, length(dBoltStd));
minClevisDepth = zeros(1, length(dBoltStd));
widthClevis = zeros(1, length(dBoltStd));

for ii = 1:length(bearingID)
    
    for loadIdx = 1:length(pLug)
    
    % Lug Diameter
    Dlug(1, ii, loadIdx) = dBoltStd(1, ii, loadIdx) + ...
        (2 * bushingThickness);
    
    % Radius of link
    Rlink(1, ii, loadIdx) = .8 * bearingOD(1, ii, loadIdx);
    
    % Defining minumum clevis depth
    minClevisDepth(1, ii, loadIdx) = Rlink(1, ii, loadIdx);
    
    % Defining clevis width
    widthClevis(1, ii, loadIdx) = (bushingFlange * 2 ) + ...
        bearingWidth(1, ii, loadIdx) + (2 *linkTol);
    
    end
    
end

%% Defining Lug Geometry

Wlug = 1.88; % DEPENEDS ON DESIGN
Rlug = .94; % DEPENEDS ON DESIGN
Theta = [atand(rxns(2,1) ./ rxns(1,1)), ...
         atand(rxns(4,1) ./ rxns(3,1)), ...
         atand(rxns(6,1) ./ rxns(5,1))];

%% Design Curves

% ----- R/D Design Curves ------
% R/D Ratio
RoDplot = linspace(.4, 3, 1000);
RoD = Rlug ./ Dlug;

% R/D design curve equation
rd = @(RoDplot) (8*RoDplot-4) ./ (3+2*RoDplot);

% ----- W/D Design Curves -----
% W/D ratio
WoDplot = linspace(.4 ,3 ,1000);
WoD = Wlug ./ Dlug;


% W/D Equation 1
wd1 = @(WoDplot) -1.1241624334181 + 1.26260602679792*(WoDplot) - ...
    0.134109402804349*(WoDplot).^2;

% W/D Equation 2
wd2 = @(WoDplot) -1.05128778218923 + 1.11850813948953*(WoDplot) - ...
    0.064994276922263*(WoDplot).^2;

% W/D Equation 3
wd3 = @(WoDplot) -1.04780311931278 + 1.09937838959058*(WoDplot) - ...
    0.050356226727686*(WoDplot).^2;

% W/D Equation 4
wd4 = @(WoDplot) -1.05695777389161 + 1.09458793219562*(WoDplot) - ...
    0.039868567021969*(WoDplot).^2;

% W/D Equation 5
wd5 = @(WoDplot) -1.04007802184702 + 1.06368514278838*(WoDplot) ...
    - 0.024983837942521*(WoDplot).^2;

% ----- Offset Loading Design Curve -----
% Offset Loading Curve
offsetLoading = @(Theta) 0.9999125874117 -  ...
    0.000129506604504392*(Theta)- 0.000091754079254061.*(Theta).^2 ...
    + 6.94250194250276E-07.*(Theta).^3;

% Creating new figure
figure(1)

% Allowing for multiple plots
hold on
% Turning grid on
grid on
% Defining grid size
grid minor

% PLotting all W/D Curves
plot(wd1(linspace(0,3, 1000)), linspace(0, 3, 1000));
plot(wd2(linspace(0,3, 1000)), linspace(0, 3, 1000));
plot(wd3(linspace(0,3, 1000)), linspace(0, 3, 1000));
plot(wd4(linspace(0,3, 1000)), linspace(0, 3, 1000));
plot(wd5(linspace(0,3, 1000)), linspace(0, 3, 1000));

% PLotting R/D curve
plot(rd(linspace(0, 3, 1000)), linspace(0, 3, 1000));

% Defining axis limits
xlim([0 2.2]);
ylim([.4 3.0]);

% Plot Descriptors
xlabel('\emph {$F_{Br}$ / $F_{Tu}$}', 'fontsize', 14, 'Interpreter', ...
    'latex');
ylabel('\emph {$\frac{R}{D}$ $\&$ $\frac{W}{D}$}', 'fontsize', 14, ...
    'Interpreter', 'latex');
title('\emph {Lug Design Chart}', 'fontsize', 16, 'Interpreter', 'latex');
legend('W/D - 1','W/D - 2','W/D - 3','W/D - 4','W/D - 5','R/D', ...
    'location','northwest');

% PLOT OFFSTE LOADING CURVE
% Creating new figure
figure(2)

% X-Vals
ThetaPlot = linspace(0, 90, 1000);
% Y-Vals
plot(ThetaPlot, offsetLoading(ThetaPlot));

%Plot adjustemnts
grid on
grid minor

% Plot Descriptors
ylabel('\emph {$P_{0}$ / $P_{\theta}$}', 'fontsize', 14, 'Interpreter', ...
    'latex');
xlabel('\emph {$\theta$ (Degrees)}', 'fontsize', 14, ...
    'Interpreter', 'latex');
title('\emph {Allowable Lateral Lug Loads}', 'fontsize', 16, ...
    'Interpreter', 'latex');
legend('Off-Axis Corrective Factor','location','northeast');
