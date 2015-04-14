%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Points2Hand_pseudo
%   Author: Kevin O'Neill
%   Date: 2015/02/27
%   Desc: 
%       This file contains pseudo-code for how to turn points into hand
%       angles. There are 27 DOFs that must be calculated to recreate hand
%       motion. Reduced and constrained models can have 23 DOFs to
%       calculate (excludes DIP since DIP and IIP are usually the same).
%
%       Finger Joint Angles (16):
%           Adb (Abduction / Aduction)
%           PIP (Proximal Interphalangeal joint)
%           IIP (Intermediate Interphalangeal joint)
%           DIP (Distal Interphalangeal joint)
%
%       Thumb Joint Angles (4):
%           Adb (Abduction / Aduction)
%           CMC (Carpometacarpal joint)            
%           MP (Metacarpophalangeal  joint)
%           IP (Interphalangeal joint)
%
%       Wrist Joint Angles(1):
%           WF (Wrist flexion/extension)sleep
%
%       Palm Degrees of Freedom (6):
%           Translation (3)
%           Rotation (3)
%
%       Marker Indicies: 23
%           One marker on each finger nail: 5
%           One marker on each finger joint: 15
%           One marker, centered, on the back of the hand: 1
%           One marker on the wrist: 1
%           One marker on the dorsal surface of the forearm: 1
%
%       Labels will follow a simplistic pattern for ease of typing and
%       reading. Each label will be a single set of (x,y,z) points in space.
%
%       Index Knuckle ( I0 )      - PIP and Adb Joint
%       Index IIP ( I1 )          - IIP Joint
%       Index DIP ( I2 )          - DIP Joint
%       Index Finger Nail ( I3 )  - Finger Tip
%
%       Repeat for each finger and thumb (T,I,M,R,L)       
%
%       Hand Back ( HB )
%
%       Wrist Joint ( WJ )
%
%       Forearm ( FA )
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load and seperate data

rotMatx = @(x) [      1,       0,        0;...
                      0, cosd(x), -sind(x);...
                      0, sind(x),  cosd(x)];
                  
rotMaty = @(x) [cosd(x),       0,  sind(x);...
                      0,       1,        0;...
               -sind(x),       0,  cosd(x)];
           
rotMatz = @(x) [cosd(x),-sind(x),        0;...
                sind(x), cosd(x),        0;...
                      0,       0,        1];

HB = [0;0;0];
I0 = [-1;1;0] + HB; M0 = [0;1;0] + HB;
I1 = rotMatz(-45)*rotMatx(45)      * [0;-1;0] .* [1;-1;1] + I0;
I2 = rotMatz(-45)*rotMatx(45+45)   * [0;-1;0] .* [1;-1;1] + I1;
I3 = rotMatz(-45)*rotMatx(45+45+30) * [0;-1;0] .* [1;-1;1] + I2;
% 
DERP = [HB';M0';I0';I1';I2';I3'];

jointAngles = [55,   45,   45,   45;
               -10,   45,   60,   30;...
               0,   10,   20,   30;...
               10,   10,   10,   10;...
               20,   30,   10,   0];
    
jointAngles(1,1) = jointAngles(1,1) -75.9638;            

ThumbMarkers = zeros(4,3);
IndexMarkers = zeros(4,3);
MiddleMarkers = zeros(4,3);
RingMarkers = zeros(4,3);
LittleMarkers = zeros(4,3);
             
for i = 1:4

    if i == 1
        ThumbMarkers(1,:)  = [-1.5, -0.5, 0] + HB';
        IndexMarkers(1,:)  = [-1, 1, 0] + HB';
        MiddleMarkers(1,:) = [ 0, 1, 0] + HB';
        RingMarkers(1,:)   = [ 1, 0.95, 0] + HB';
        LittleMarkers(1,:) = [ 2, 0.80, 0] + HB';
    else
        ThumbMarkers(i,:)  = (rotMatz(jointAngles(1,1))*rotMatx(sum(jointAngles(1,2:i))) * [0;-1;0] .* [1;-1;1] + ThumbMarkers(i-1,:)')';
        IndexMarkers(i,:)  = (rotMatz(jointAngles(2,1))*rotMatx(sum(jointAngles(2,2:i))) * [0;-1;0] .* [1;-1;1] + IndexMarkers(i-1,:)')';
        MiddleMarkers(i,:) = (rotMatz(jointAngles(3,1))*rotMatx(sum(jointAngles(3,2:i))) * [0;-1;0] .* [1;-1;1] + MiddleMarkers(i-1,:)')';
        RingMarkers(i,:)   = (rotMatz(jointAngles(4,1))*rotMatx(sum(jointAngles(4,2:i))) * [0;-1;0] .* [1;-1;1] + RingMarkers(i-1,:)')';
        LittleMarkers(i,:) = (rotMatz(jointAngles(5,1))*rotMatx(sum(jointAngles(5,2:i))) * [0;-1;0] .* [1;-1;1] + LittleMarkers(i-1,:)')';
        
    end % END IF
    
end % END FOR
    
% hold on
% 
% scatter3(0,0,0, 'k');
% plot3([0, ThumbMarkers(1,1)], [0, ThumbMarkers(1,2)], [0,ThumbMarkers(1,3)], 'k')
% plot3([0, IndexMarkers(1,1)], [0, IndexMarkers(1,2)], [0,IndexMarkers(1,3)], 'k')
% plot3([0, MiddleMarkers(1,1)], [0, MiddleMarkers(1,2)], [0,MiddleMarkers(1,3)], 'k')
% plot3([0, RingMarkers(1,1)], [0, RingMarkers(1,2)], [0,RingMarkers(1,3)], 'k')
% plot3([0, LittleMarkers(1,1)], [0, LittleMarkers(1,2)], [0,LittleMarkers(1,3)], 'k')
% 
% scatter3(ThumbMarkers(:,1), ThumbMarkers(:,2), ThumbMarkers(:,3), 'r'); 
% plot3(ThumbMarkers(:,1), ThumbMarkers(:,2), ThumbMarkers(:,3), 'r')
% 
% scatter3(IndexMarkers(:,1), IndexMarkers(:,2), IndexMarkers(:,3), 'g'); 
% plot3(IndexMarkers(:,1), IndexMarkers(:,2), IndexMarkers(:,3), 'g')
% 
% scatter3(MiddleMarkers(:,1), MiddleMarkers(:,2), MiddleMarkers(:,3), 'b'); 
% plot3(MiddleMarkers(:,1), MiddleMarkers(:,2), MiddleMarkers(:,3), 'b')
% 
% scatter3(RingMarkers(:,1), RingMarkers(:,2), RingMarkers(:,3), 'm');
% plot3(RingMarkers(:,1), RingMarkers(:,2), RingMarkers(:,3), 'm')
% 
% scatter3(LittleMarkers(:,1), LittleMarkers(:,2), LittleMarkers(:,3), 'c'); 
% plot3(LittleMarkers(:,1), LittleMarkers(:,2), LittleMarkers(:,3), 'c')
% hold off
                
%% Finger Angles (Not Knuckle)

% Allocate variable to store angles
calcAngles = zeros(5,4);

% Create FingerMarkers matrix
FingerMarkers = [ThumbMarkers;IndexMarkers;MiddleMarkers;RingMarkers;LittleMarkers];

% Calculate the vectors from one joint to another
vecFingers = FingerMarkers(2:end,:) - FingerMarkers(1:end-1,:);
vecFingers(~mod(1:length(vecFingers), 4)',:) = []; % Cull excess vectors (eg: IndexTip to Middle Knuckle)

% Create an index variable 
idx1 = 1:length(vecFingers);
idx1(~mod(idx1,3)) = [];
idx2 = idx1+1;

% Define a function [vecMag] such that it returns the magnitude of a matrix
% [x] of vectors [x_i].
vecMag = @(x) sqrt(x(:,1).^2 + x(:,2).^2 + x(:,3).^2);

% Allocate variable to store calculated angles between distal bones of the
% fingers. (2 joints for 5 fingers -> 10 indicies)
anglesTmp = zeros(1,10);

%  Calculate the angle for each distal and intermediate joint of the finger
% theta = acosd( dot(v,w) / (||v|| * ||w||));
anglesTmp = acosd( dot(vecFingers(idx1,:),vecFingers(idx2,:),2) ./ (vecMag(vecFingers(idx1,:)).*vecMag(vecFingers(idx2,:))));

% Store the calculated angles in the [calcAngles] matrix. Use the [real]
% values as imaginary values occur when the angle is 0 (or 180);
calcAngles(:,3:4) = real(reshape(anglesTmp',2,5)');

%% Kuckle Angles

% Create a matrix of knuckle positions;
knuckleMarkers = FingerMarkers(fliplr(~mod(1:length(FingerMarkers),4)),:);

% Create a matrix of vectors from the [HB] to the knuckles
vecHandBack = knuckleMarkers - repmat(HB', 5,1);

% Create a matrix of vectors from one knuckle to another. (T->I, I->M, ...)
vecNeighbor = knuckleMarkers(2:end,:) - knuckleMarkers(1:end-1,:);
vecNeighbor = [vecNeighbor; vecNeighbor(end,:)]; % Add a copy of the last vector for the little (5th) finger

% Calculate the normals created by <knuckle-to-HB> and <knuckle-to-knuckle>
planeNormals = cross(vecNeighbor, vecHandBack,2);   
planeNormals2 = planeNormals./repmat(vecMag(planeNormals),1,3);
% virtualMarkers = knuckleMarkers + planeNormals2;

% Generate a vector normal to the projection plane.
normProj2 = cross(planeNormals2, vecNeighbor, 2);
normProj2 = normProj2 ./ repmat(vecMag(normProj2),1,3); % Normalize result

% Project the '0' angle markers onto the hand. The Thumb zero is off by an offset
normPoint = normProj2 + knuckleMarkers;
hold on
scatter3(normPoint(:,1), normPoint(:,2), normPoint(:,3), 10, repmat([1, 0, 1],5,1))
hold off

% Calculate the projection of the proximal phalange onto the HB plane
% normal [planeNormals2] and knuckle vector [vecNeighbor]. 
% sind(theta) = (dot(v, w) / ||w||^2) * w
normProj = repmat((dot(vecFingers(fliplr(~mod(1:length(vecFingers),3)),:), planeNormals2,2)./(vecMag(planeNormals2).^2)),1,3).*planeNormals2;
latProj  = repmat((dot(vecFingers(fliplr(~mod(1:length(vecFingers),3)),:), vecNeighbor,2)  ./(vecMag(vecNeighbor).^2))  ,1,3).*vecNeighbor;

% Calculate the angles and sign of angle for the projdections
% theta = asind((dot(v, w) / ||w||^2) * w)
% angleSign = -sign(<>)
calcAngles(:,2) = asind(vecMag(normProj)) .* -sign(normProj(:,3));
calcAngles(:,1) = asind(vecMag( latProj)) .* -sign( latProj(:,1));


%% Index joint angles
% IndexAngles = zeros(4,1); % Abd, PIP, IIP, DIP
% 
% % Index DIP Angle
% vec1 = I3 - I2; % DIP to tip vector
% vec2 = I2 - I1; % IIP to DIP vector
% 
% IndexAngles(4) = acosd( dot(vec1, vec2) / (norm(vec1)*norm(vec2)) ); % Finds the angle between two vectors.
% 
% % Index IIP Angle
% vec1 = I2 - I1; % IIP to DIP vector
% vec2 = I1 - I0; % Knuckle to IIP vector
% 
% IndexAngles(3) = acosd( dot(vec1, vec2) / (norm(vec1)*norm(vec2)) ); % Finds the angle between two fectors.

%%  Index PIP Angle
% This one is more tricky. We need to define a plane and project the
% <I1-I0> vector onto it. The sine of the magnitude of the projection is
% the PIP angles and the angle between the projection and the axis is the
% Abd angle. This plane will be created by first creating a 'hand plane'
% using two knuckle joints and the HB point. Then celculating the normal to
% that plane. Creating a virtual point 'above' the knuckle of interest,
% then solving for a new plane using the virtual point, the knuckle of
% interest and a neighboring knuckle.

% The index knuckle will be our plane "anchor"
% planeNormal = cross(I0 - HB,I0 - M0);
% planeNormal = planeNormal./norm(planeNormal);
% 
% virtualIndex = I0 + planeNormal;
% 
% projNormal = cross(I0 - virtualIndex, I0 - M0);
% projNormal = projNormal./norm(projNormal);
% 
% proj2 = I1 - dot(I1-I0, projNormal)*projNormal-I0;
% 
% IndexAngles(2) = asind(norm(proj2)) * -sign(proj2(3)); % PIP angle
% IndexAngles(1) = acosd( dot(proj2, [0, 0, 1]) / (norm(proj2)*norm([0, 0, 1])) )-180; % Abd angle

%% Method 2

% planeNormal = cross(I0 - HB,I0 - M0);
% planeNormal = planeNormal./norm(planeNormal);
% virtualIndex = I0 + planeNormal;
% 
% vec1 = planeNormal;
% vec2 = M0-I0;
% vec2 = vec2./norm(vec2);
% 
% normProj2 = cross(vec1, vec2);
% vec3 = normProj2./norm(normProj2);
% 
% normProj = (dot(I1-I0, vec1)/norm(vec1)^2)*vec1;
% latProj =  (dot(I1-I0, vec2)/norm(vec2)^2)*vec2;
% testProj = (dot(I1-I0, vec3)/norm(vec3)^2)*vec3;
% 
% 
% combined = normProj + latProj;
% 
% IndexAngles(2) = asind(norm(normProj)) * -sign(normProj(3)); % PIP angle
% IndexAngles(1) = asind(norm( latProj)) * -sign( latProj(1)); % Abd angle

% basisPlane=null([1,1,0]./norm([1,1,0])); %basis for the plane
% 
% basisCoefficients= bsxfun(@minus, I1 ,I0)'*basisPlane ;
% 
% theResult=bsxfun(@plus,basisCoefficients*basisPlane.', I0);

%% Thumb Angles

%% Wrist Angle

%% Palm/Hand Orientation in space

translation = HB; % Maybe plus some offset
handNorm = cross(HB-I0, HB-M0);
handNorm = handNorm./vecMag(handNorm);
handTail = HB-WF; % Vector from hand back to wrist joint 
globalNorm = eye(3);

% Projection of handNorm onto global x-axis 
vecRotX = asind((dot(handNorm, globalNorm(1,:)) / vecMag(globalNorm(1,:))^2) * globalNorm(1,:)); 
vecRotY = asind((dot(handNorm, globalNorm(2,:)) / vecMag(globalNorm(2,:))^2) * globalNorm(2,:)); 
vecRotZ = acosd(dot(handTail, globalNorm(2,:)) / (norm(handTail) * norm(globalNorm(2,:))));
% Compare coordinate system to cross(HB-I0, HB-M0) with HB->WF

% Decompose a rotation matrix: http://nghiaho.com/?page_id=846

thetax = atan2(R(3,2), R(3,3));
thetay = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
thetaz = atan2(R(2,1), R(1,1));
% EOF