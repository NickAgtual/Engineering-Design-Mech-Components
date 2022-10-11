
x = .75:.1:3;
y = x;

[xPlot, yPlot] = meshgrid(x, y);

fs = 2.5;

us = 77000; % DEPENDS ON ALUMINUM MATERIAL SELECTION

D = .435; % DEPENDS ON BOLT SELECTION

cf = 3;

load = sqrt(rxns(3,1)^2 + rxns(4,1)^2); %LATERAL LOAD
LoadAngle = atan(rxns(4,1), rxns(3,1));

% ----- R/D Design Curves ------
% R/D Ratio
RoD = xPlot ./ D;

% R/D design curve equation
rd = @(RoDplot) (8*RoDplot-4) ./ (3+2*RoDplot);

% ----- W/D Design Curves -----
% W/D ratio
WoD = yPlot ./ D;


% W/D Equation 1
wd1 = @(WoDplot) -1.1241624334181 + 1.26260602679792*(WoDplot) - ...
    0.134109402804349*(WoDplot).^2;

fac1 = zeros(size(RoD, 1), size(RoD, 2));


for ii = 1:size(RoD, 1)
    for jj = 1:size(RoD, 2)
        
        if wd1(WoD(ii, jj)) > rd(RoD(ii, jj))
            
            fac1(ii, jj) = rd(RoD(ii, jj));
            
        else
            fac1(ii, jj) = wd1(WoD(ii, jj));
            
        end
    end
end

offsetLoading = @(LoadAngle) 0.9999125874117 -  ...
    0.000129506604504392*(LoadAngle)- 0.000091754079254061.*(LoadAngle).^2 ...
    + 6.94250194250276E-07.*(LoadAngle).^3;


syms thick

thickness = zeros(size(RoD, 1), size(RoD, 2));

for ii = 1:size(RoD, 1)
    for jj = 1:size(RoD, 2)
        
        sbr = us * fac1(ii, jj) *  ...
            offsetLoading(atand(rxns(4,1) ./ rxns(3,1)));
        
        sigma = sbr ./ fs;
        
        t = sigma == (.6 * load * ...
            cf) ./ (D * thick);
        
        thickness(ii, jj) = double(solve(t, thick));
        
    end
end
figure(3)

surfc(xPlot, yPlot, double(thickness))

xlabel('Lug Radius');
ylabel('Lug Width');
zlabel('Lug Thickness');
title('Lateral Lug Parametric Study');

xlabel('\emph {Lug Radius (in)}', 'fontsize', 14, ...
    'Interpreter', 'latex');
ylabel('\emph {Lug Width (in)}', 'fontsize', 14, ...
    'Interpreter', 'latex');
zlabel('\emph {Lug Thickness (in)}', 'fontsize', 14, ...
    'Interpreter', 'latex');
title('\emph {Lateral Lug Parametric Study}', ...
    'fontsize', 16, 'Interpreter', 'latex');



