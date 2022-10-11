%% Defining Lug & Bolt Geometry


W = 1.88; % DEPENEDS ON DESIGN
R = 1; % DEPENEDS ON DESIGN
T = atand(rxns(4,1) ./ rxns(3,1));% DEPENEDS ON DESIGN



D = .435;

%% Design Curves

% ----- R/D Design Curves ------
% R/D Ratio
RoDplot = linspace(.4, 3, 1000);
RoD = R ./ D;

% R/D design curve equation
rd = @(RoDplot) (8*RoDplot-4) ./ (3+2*RoDplot);

% ----- W/D Design Curves -----
% W/D ratio
WoDplot = linspace(.4 ,3 ,1000);
WoD = W ./ D;


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

%% Bearing Strength

% Extracting ultimate strenghts from aluminum alloys table (kpsi)
ultimateStrength = table2array(aluminumAlloys(1:end, 3))';

% Ultimate strenght of aluminum alloy for clevis (psi)
Sut = ultimateStrength * 1000;

% Defining Sbr as symbolic variable
syms Sbr

% Initializing size changing array
bearingStrength = zeros(length(ultimateStrength), 1);

for ii = 1:length(ultimateStrength)
    
    if wd1(WoD) > rd(RoD)
        
        % Bearing strength equaiton
        bearingStrenghtEq = Sbr == Sut(ii) * wd1(WoD) * ...
            offsetLoading(T);
        
        % Solving for bearing strength
        bearingStrength(ii) = double(solve(bearingStrenghtEq, ...
            Sbr));
        
    else
        
        % Bearing strength equaiton
        bearingStrenghtEq = Sbr == Sut(ii) * rd(RoD) * ...
            offsetLoading(T);
        
        % Solving for bearing strength
        bearingStrength(ii) = double(solve(bearingStrenghtEq, ...
            Sbr));
        
    end
end

%% Calculating Thickness of Lug

% Defini tLug as symbolic variable
syms tLug

% Initializing size changing arrays
sigmaLug = zeros(length(ultimateStrength),1);
LT = zeros(length(ultimateStrength),1);

for ii = 1:length(ultimateStrength)
        
        % Lug stress equation
        sigmaLug(ii) = bearingStrength(ii) ./ designFactorLug;
        
        % Lug thickness equation
        lugThicknessEq = sigmaLug(ii) == (.6 * pLug(2) * ...
            crashFactor) ./ (D * tLug);
        
        % Solving for lug thickness
        LT(ii) = ...
            double(solve(lugThicknessEq, tLug));
        
end
