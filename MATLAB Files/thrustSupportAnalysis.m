%% Lateral Support Analysis

designFactorLug = 2.5;
crashFactor = 3;
thrustLoad = 3.1762e+03;

% Reading Excel File with Strenth Values
% Row 1 is length grain
% Row 2 is long transverse grain
strengthAluminum = [ 74000 74000 68000 77000; ...
                     78000 75000 69000 78000];

% Using SAE Grade 5 Bolt
lugDiameterThrust = .56;
loadAngleThrust = 71.1792;

% Adjustable Parameters
lugRadiusThrust = .82;
lugWidthThrust = 1.64;
lugAxisThrust = 70.34; % From Design

thrustAngle = loadAngleThrust + lugAxisThrust;

% Defining RoD and WoD ratios
RoDaft= lugRadiusThrust ./ lugDiameterThrust;
WoDaft = lugWidthThrust ./ lugDiameterThrust;

% R/D design curve equation
rd = @(RoDplot) (8*RoDplot-4) ./ (3+2*RoDplot);

% ----- W/D Design Curves -----
% W/D ratio
WoDplot = linspace(.4 ,3 ,1000);


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

% Defining Sbr as symbolic variable
syms Sbr

% Initializing size changing vectors
bearingStrengthThrust = zeros(size(strengthAluminum, 1), ...
    length(strengthAluminum));

for ii = 1:size(strengthAluminum, 1)

    for jj = 1:length(strengthAluminum)

        if ii == 1

            if wd1(WoDaft) > rd(RoDaft)

                % Bearing strength equaiton
                bearingStrenghtEqAft = Sbr == ...
                    strengthAluminum(ii, jj) * wd1(WoDaft) * ...
                    offsetLoading(thrustAngle);

                % Solving for bearing strength
                bearingStrengthThrust(ii, jj) = ...
                    double(solve(bearingStrenghtEqAft, Sbr));

            else

                % Bearing strength equaiton
                bearingStrenghtEqAft = Sbr == ...
                    strengthAluminum(ii, jj) * rd(RoDaft) * ...
                    offsetLoading(thrustAngle);

                % Solving for bearing strength
                bearingStrengthThrust(ii, jj) = ...
                    double(solve(bearingStrenghtEqAft, Sbr));

            end

        else

            if wd2(WoDaft) > rd(RoDaft)

                % Bearing strength equaiton
                bearingStrenghtEqAft = Sbr ==  ...
                    strengthAluminum(ii, jj) * wd1(WoDaft) * ...
                    offsetLoading(thrustAngle);

                % Solving for bearing strength
                bearingStrengthThrust(ii, jj) = ...
                    double(solve(bearingStrenghtEqAft, Sbr));

            else

                % Bearing strength equaiton
                bearingStrenghtEqAft = Sbr == ... 
                    strengthAluminum(ii, jj) * rd(RoDaft) * ...
                    offsetLoading(thrustAngle);

                % Solving for bearing strength
                bearingStrengthThrust(ii, jj) = ...
                    double(solve(bearingStrenghtEqAft, Sbr));

            end

        end
    end
end


%% Calculating Thickness of Lug

% Defini tLug as symbolic variable
syms tLug

% Initializing Size Changing Vectors
lugThicknessThrust = zeros(size(strengthAluminum, 1), ...
    length(strengthAluminum));

for ii = 1:size(strengthAluminum, 1)

    for jj = 1:length(strengthAluminum)

        % Lug stress equation
        sigmaLugThrust = bearingStrengthThrust(ii, jj) ./ designFactorLug;

        % Lug thickness equation
        lugThicknessEq = sigmaLugThrust == ((.6 * thrustLoad) * ...
            crashFactor) ./ (lugDiameterThrust * tLug);

        % Solving for lug thickness
        lugThicknessThrust(ii, jj) = ...
            double(solve(lugThicknessEq, tLug));

    end
end

GrainDirection = ["Longitudinal Grain" ; "Long Transverse"];
AL7075T62 = [lugThicknessThrust(1, 1); lugThicknessThrust(2, 1) ];
AL7075T651 = [lugThicknessThrust(1, 2); lugThicknessThrust(2, 2) ];
AL7075T7351 = [lugThicknessThrust(1, 3); lugThicknessThrust(2, 3) ];
AL7075T7651 = [lugThicknessThrust(1, 4); lugThicknessThrust(2, 4) ];

resultsThrustThickness = table(GrainDirection, AL7075T62, AL7075T651, ...
    AL7075T7351, AL7075T7651);

fprintf('\n');

disp(resultsThrustThickness);

%% Calculating True Factor of Safety

% DEPENDS ON DESIGN
thrustThicknessActual = .25;

syms FSActThrust

sigmaThrustLugActual = bearingStrengthThrust(1, 2) ./ FSActThrust;

FSeqThrust = sigmaThrustLugActual == ((.6 * thrustLoad) * ...
            crashFactor) ./ (lugDiameterThrust * thrustThicknessActual);
        
thrustFS =  double(solve(FSeqThrust, FSActThrust)); 

fprintf('Thrust Support F.S. = %f \n', thrustFS);



