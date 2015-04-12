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

jointAngles = [55,   45,   90,  90;
               -10,   45,   80,   90;...
               0,   45,   70,   90;...
               10,   45,   50,   90;...
               20,   45,   45,   90];
    
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
    
hold on

scatter3(0,0,0, 'k');
plot3([0, ThumbMarkers(1,1)], [0, ThumbMarkers(1,2)], [0,ThumbMarkers(1,3)], 'k')
plot3([0, IndexMarkers(1,1)], [0, IndexMarkers(1,2)], [0,IndexMarkers(1,3)], 'k')
plot3([0, MiddleMarkers(1,1)], [0, MiddleMarkers(1,2)], [0,MiddleMarkers(1,3)], 'k')
plot3([0, RingMarkers(1,1)], [0, RingMarkers(1,2)], [0,RingMarkers(1,3)], 'k')
plot3([0, LittleMarkers(1,1)], [0, LittleMarkers(1,2)], [0,LittleMarkers(1,3)], 'k')

scatter3(ThumbMarkers(:,1), ThumbMarkers(:,2), ThumbMarkers(:,3), 'r'); 
plot3(ThumbMarkers(:,1), ThumbMarkers(:,2), ThumbMarkers(:,3), 'r')

scatter3(IndexMarkers(:,1), IndexMarkers(:,2), IndexMarkers(:,3), 'g'); 
plot3(IndexMarkers(:,1), IndexMarkers(:,2), IndexMarkers(:,3), 'g')

scatter3(MiddleMarkers(:,1), MiddleMarkers(:,2), MiddleMarkers(:,3), 'b'); 
plot3(MiddleMarkers(:,1), MiddleMarkers(:,2), MiddleMarkers(:,3), 'b')

scatter3(RingMarkers(:,1), RingMarkers(:,2), RingMarkers(:,3), 'm');
plot3(RingMarkers(:,1), RingMarkers(:,2), RingMarkers(:,3), 'm')

scatter3(LittleMarkers(:,1), LittleMarkers(:,2), LittleMarkers(:,3), 'c'); 
plot3(LittleMarkers(:,1), LittleMarkers(:,2), LittleMarkers(:,3), 'c')
hold off
                
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
% normPoint = normProj2 + knuckleMarkers;
% hold on
% scatter3(normPoint(:,1), normPoint(:,2), normPoint(:,3), 10, repmat([1, 0, 1],5,1))
% hold off

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


%% Rotation Matrix decomposition: Tait-Bryan Angles
% Bone basis orientation:
%   x-axis: pointing 'down' out of the bone if the hand is palm down.
%   y-axis: point 'right' out of the bone if the hand is palm down.
%   z-axis: along bone from proximal end to distal end.

calcAngles2 = zeros(5,4);

rotAngles = @(x) [(atan2(x(3,2,:), x(3,3,:)) .* (180/pi)),...
                  -acos(x(3,1,:)).*(180/pi),...  %(atan2(-x(3,1,:), sqrt(x(3,2,:).^2 + x(3,3,:).^2)) .* (180/pi)),...
                  (atan2(x(2,1,:), x(1,1,:)) .* (180/pi))];
              
rotMat = zeros(3,3,5*3); % One rotation matrix for each joint for each finger. Thumb MC, IIP, PIP, Index MC,...

FingerMarkers = [ThumbMarkers;IndexMarkers;MiddleMarkers;RingMarkers;LittleMarkers];
% FingerMarkers = FingerMarkers + repmat([5,6,7], length(FingerMarkers),1);
% Calculate the vectors from one joint to another
vecFingers = FingerMarkers(2:end,:) - FingerMarkers(1:end-1,:);
vecFingers(~mod(1:length(vecFingers), 4)',:) = []; % Cull excess vectors (eg: IndexTip to Middle Knuckle)
tic

% Create hand orientation basis
% Wrist to knuckles (z-axis)
WF = HB' + [0,-1,0];
handz = HB'-WF;
handz = handz./norm(handz);

% Middle knuckle and Index knuckle, respectivly
handx = cross(HB'-FingerMarkers(5,:), HB'-FingerMarkers(9,:));
handx = handx./norm(handx);

handy = cross(handx,handz);
handy = handy./norm(handy);

handRot = [handx', handy', handz'];

handRotDecom = rotAngles(handRot);

% Create Normals between bones. Ex: 
% Right Hand, Index
%   cross product Intermediate Phalange with Proximal Phalange to make a normal
%   pointing to the right (if the hand is palm down): y-axis of bone basis
jointNormal1 = cross(vecFingers(2:end,:), vecFingers(1:end-1,:),2);   
jointNormal1 = jointNormal1./repmat(vecMag(jointNormal1),1,3);

jointNormal1(end+1,:) = jointNormal1(end,:);

% Remove the Distal Phalange to Proximal Phalange calculation and replaces
% it with a copy of the Intermediate Phalange to Distal Phalange
% calculation. All three joints in a finger will have the same first normal
% due to the constraints in rotation of the fingers (No finger roll).
cullIdx = ~mod(1:length(jointNormal1),3);
jointNormal1(cullIdx, :) = jointNormal1(fliplr(cullIdx), :);

% Index of negative flexions (extensions)
negIdx = ~(sign(dot(jointNormal1, repmat(handy,length(jointNormal1),1), 2))+1);

jointNormal1(negIdx,:) = jointNormal1(negIdx,:) .*-1;

% Create the second normal (x-axis bone basis) for each joint.
jointNormal2 = cross(vecFingers, jointNormal1,2);
jointNormal2 = jointNormal2./repmat(vecMag(jointNormal2),1,3);

% Construct Rotation matrix for each bone
rotMat = [permute(jointNormal2', [1,3,2]),...
          permute(jointNormal1', [1,3,2]),...
          permute(vecFingers',   [1,3,2])];

rotMatDecom = permute(rotAngles(rotMat), [3,2,1]);

% handCenteredRot = rotMatDecom + repmat(handRotDecom, length(rotMatDecom),1);
handCenteredRot = rotMatDecom + repmat([0,90,90], length(rotMatDecom),1);
fingerCenteredRot = handCenteredRot(2:end,:) - handCenteredRot(1:end-1,:);
fingerCenteredRot = fingerCenteredRot(~~mod(1:length(fingerCenteredRot),3),:);

calcAngles2(:,[3,4]) = real(reshape(fingerCenteredRot(:,2)',2,5)');
calcAngles2(:,[2,1]) = handCenteredRot(fliplr(~mod(1:length(handCenteredRot),3)),2:3).*-1;

toc

%%

% testx = rotMat(:,:,14) * rotMat(:,1,15);
% testy = rotMat(:,:,14) * rotMat(:,2,15);
% testz = rotMat(:,:,14) * rotMat(:,3,15);
test1x = rotMat(:,1,10) * rotMat(:,1,11)' * rotMat(:,1,11);
test1y = rotMat(:,2,10) * rotMat(:,2,11)' * rotMat(:,2,11);
test1z = rotMat(:,3,10) * rotMat(:,3,11)' * rotMat(:,3,11);


rotAngles([test1x, test1y, test1z])

%% Plot Result

% calcAngles2(end,end) = 90

for i = 1:4

    if i == 1
        ThumbMarkers2(1,:)  = [-1.5, -0.5, 0] + HB';
        IndexMarkers2(1,:)  = [-1, 1, 0] + HB';
        MiddleMarkers2(1,:) = [ 0, 1, 0] + HB';
        RingMarkers2(1,:)   = [ 1, 0.95, 0] + HB';
        LittleMarkers2(1,:) = [ 2, 0.80, 0] + HB';
    else
        ThumbMarkers2(i,:)  = (rotMatz(calcAngles2(1,1))*rotMatx(sum(calcAngles2(1,2:i))) * [0;-1;0] .* [1;-1;1] + ThumbMarkers2(i-1,:)')';
        IndexMarkers2(i,:)  = (rotMatz(calcAngles2(2,1))*rotMatx(sum(calcAngles2(2,2:i))) * [0;-1;0] .* [1;-1;1] + IndexMarkers2(i-1,:)')';
        MiddleMarkers2(i,:) = (rotMatz(calcAngles2(3,1))*rotMatx(sum(calcAngles2(3,2:i))) * [0;-1;0] .* [1;-1;1] + MiddleMarkers2(i-1,:)')';
        RingMarkers2(i,:)   = (rotMatz(calcAngles2(4,1))*rotMatx(sum(calcAngles2(4,2:i))) * [0;-1;0] .* [1;-1;1] + RingMarkers2(i-1,:)')';
        LittleMarkers2(i,:) = (rotMatz(calcAngles2(5,1))*rotMatx(sum(calcAngles2(5,2:i))) * [0;-1;0] .* [1;-1;1] + LittleMarkers2(i-1,:)')';
        
    end % END IF
    
end % END FOR
    
hold on

scatter3(0,0,0, 'k');
plot3([0, ThumbMarkers2(1,1)], [0, ThumbMarkers2(1,2)], [0,ThumbMarkers2(1,3)], 'k')
plot3([0, IndexMarkers2(1,1)], [0, IndexMarkers2(1,2)], [0,IndexMarkers2(1,3)], 'k')
plot3([0, MiddleMarkers2(1,1)], [0, MiddleMarkers2(1,2)], [0,MiddleMarkers2(1,3)], 'k')
plot3([0, RingMarkers2(1,1)], [0, RingMarkers2(1,2)], [0,RingMarkers2(1,3)], 'k')
plot3([0, LittleMarkers2(1,1)], [0, LittleMarkers2(1,2)], [0,LittleMarkers2(1,3)], 'k')

scatter3(ThumbMarkers2(:,1), ThumbMarkers2(:,2), ThumbMarkers2(:,3), 'r'); 
plot3(ThumbMarkers2(:,1), ThumbMarkers2(:,2), ThumbMarkers2(:,3), 'r')

scatter3(IndexMarkers2(:,1), IndexMarkers2(:,2), IndexMarkers2(:,3), 'g'); 
plot3(IndexMarkers2(:,1), IndexMarkers2(:,2), IndexMarkers2(:,3), 'g')

scatter3(MiddleMarkers2(:,1), MiddleMarkers2(:,2), MiddleMarkers2(:,3), 'b'); 
plot3(MiddleMarkers2(:,1), MiddleMarkers2(:,2), MiddleMarkers2(:,3), 'b')

scatter3(RingMarkers2(:,1), RingMarkers2(:,2), RingMarkers2(:,3), 'm');
plot3(RingMarkers2(:,1), RingMarkers2(:,2), RingMarkers2(:,3), 'm')

scatter3(LittleMarkers2(:,1), LittleMarkers2(:,2), LittleMarkers2(:,3), 'c'); 
plot3(LittleMarkers2(:,1), LittleMarkers2(:,2), LittleMarkers2(:,3), 'c')
hold off

%% Plotting

globalBasis = [[-1;0;0],[0;1;0],[0;0;1]];
% handBasis = [[0.5774;0.5774;0.5774],[-0.5774;0.5774;0.5774],[0;- 0.7071; 0.7071]];
handBasis = [[0;0;-1],[-1;0;0],[0;1; 0]];

r = [-90,0,0];
rotTest = [cosd(r(3)),-sind(r(3)),0;sind(r(3)),cosd(r(3)),0;0,0,1]*...
          [cosd(r(2)),0,sind(r(2));0,1,0;-sind(r(2)),0,cosd(r(2))]*...
          [1,0,0;0,cosd(r(1)),-sind(r(1));0,sind(r(1)),cosd(r(1))];
testMat = [rotTest*globalBasis(:,1), rotTest*globalBasis(:,2), rotTest*globalBasis(:,3)];
      
figure(2)
hold on
scatter3(globalBasis(1,:),globalBasis(2,:),globalBasis(3,:), 10, [0,0,0])

scatter3(handBasis(1,:),handBasis(2,:),handBasis(3,:), 20, [1,0,0])
scatter3(testMat(1,:),testMat(2,:),testMat(3,:), 20, [0,0,1])

for i = 1:3
    plot3([globalBasis(1,i),0],[globalBasis(2,i),0],[globalBasis(3,i),0], 'k');
    plot3([handBasis(1,i),0],  [handBasis(2,i),0],  [handBasis(3,i),0],   'r');
    plot3([testMat(1,i),0],    [testMat(2,i),0],    [testMat(3,i),0],     'b');
end % END FOR

text(testMat(1,1),testMat(2,1),testMat(3,1), 'x')
text(testMat(1,2),testMat(2,2),testMat(3,2), 'y')
text(testMat(1,3),testMat(2,3),testMat(3,3), 'z')

text(globalBasis(1,1),globalBasis(2,1),globalBasis(3,1), 'x')
text(globalBasis(1,2),globalBasis(2,2),globalBasis(3,2), 'y')
text(globalBasis(1,3),globalBasis(2,3),globalBasis(3,3), 'z')

xlabel('x')
ylabel('y')
zlabel('z')

xlim([-1,1])
ylim([-1,1])
zlim([-1,1])

rotAngles(handBasis)
rotAngles(testMat)

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

% translation = HB; % Maybe plus some offset
% handNorm = cross(HB-I0, HB-M0);
% handNorm = handNorm./vecMag(handNorm);
% handTail = HB-WF; % Vector from hand back to wrist joint 
% globalNorm = eye(3);
% 
% % Projection of handNorm onto global x-axis 
% vecRotX = asind((dot(handNorm, globalNorm(1,:)) / vecMag(globalNorm(1,:))^2) * globalNorm(1,:)); 
% vecRotY = asind((dot(handNorm, globalNorm(2,:)) / vecMag(globalNorm(2,:))^2) * globalNorm(2,:)); 
% vecRotZ = acosd(dot(handTail, globalNorm(2,:)) / (norm(handTail) * norm(globalNorm(2,:))));
% % Compare coordinate system to cross(HB-I0, HB-M0) with HB->WF
% 
% % Decompose a rotation matrix: http://nghiaho.com/?page_id=846
% 
% thetax = atan2(R(3,2), R(3,3)) * (180/pi);
% thetay = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2)) * (180/pi);
% thetaz = atan2(R(2,1), R(1,1)) * (180/pi);
% EOF