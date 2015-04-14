%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Torus Speed Control
%       Defines a speed profile for the torus task in the human PNI project
%       Author: Kevin O'Neill
%       Date: 2015/03/22
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%                          Plateau
%                     ________________
%                    /                \
%          Rise     /                  \    Fall
%                  /                    \
%   ______________/                      \________________________
%
%
%


%% Speed Profile Variables

plateauLength = 200;    % [miliseconds] Length of plateau
riseLength    = 900;    % [miliseconds] Length of rising section
fallLength    = 900;    % [miliseconds] Length of falling section
time          = 1:2000; % [miliseconds] Total Time

%%% (plateauLength + riseLength + fallLength) <= length(time) %%%

maxSpeed      = 1;      % [arbitrary] Maximum Speed of function

%% Sigmoid Function Creation

tRise = linspace(-6,6,riseLength); % Convert time to something easier for the math to compute.
SRise = 1./(1+exp(-tRise));
SRise = SRise.*maxSpeed;

tFall = linspace(-6,6,fallLength); % Convert time to something easier for the math to compute.
SFall = 1./(1+exp(-tFall));
SFall = fliplr(SFall).*maxSpeed;

tPlat = 1:plateauLength;
SPlat = ones(1,length(tPlat)) .* maxSpeed;

%% Combine Functions

timeLeft = length(time) - plateauLength - riseLength - fallLength;

tBase1 = zeros(1, floor(timeLeft/2));
tBase2 = zeros(1, floor(timeLeft/2));

SFinal = [tBase1(:); SRise(:); SPlat(:); SFall(:); tBase2(:)];

plot(SFinal)

% EOF