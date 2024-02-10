% RBE 501 - Robot Dynamics - Spring 2023
% Homework 4, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 03/20/2023
clear, clc, close all
addpath('utils');

plotOn = true; 
nTests = 10;

%% Create the manipulator
mdl_stanford
stanf

if plotOn
   stanf.teach(zeros(1,6)); 
end

%% YOUR CODE HERE
L1 = 0.412;
L2 = 0.154;
L4= 0.263;

qlim = [-pi/2  pi/2;  % q(1)
        -pi/4  pi/2;  % q(2)
         pi/3 pi/3;   %q(3)
        -pi/4  pi/2;  %q(4)
         pi/4  pi/2;  %q(5)
         pi/6  pi/2]; %q(6) 



%% Part A - Forward Kinematics via PoE in the body frame
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S_body = [0 0 1 -cross([0 0 1],[L2 0 -L4-L1]);
     -1 0 0 -cross([ -1 0 0],[-L2 0 -L4]);
      0 0 0  0 0 1;
      0 0 1 -cross([0 0 1],[0 0 -L4]);
      0 1 0 -cross([0 1 0],[0 0 -L4]);
      0 0 1 -cross([0 0 1],[0 0 -L4])]';

% Let us calculate the homogeneous transformation matrix M for the
% home configuration
R_home = [0 -1 0; 1 0 0; 0 0 1]';
t_home = [0 L2 L1+L4]';
M = [R_home t_home; 0 0 0 1];

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 10 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(2,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(3,2) - qlim(6,1)) * rand()];
    
    % Calculate the forward kinematics
     T = fkine(S_body,M,q,'body');
    
    if plotOn
        stanf.teach(q);
        title('Forward Kinematics Test');
    end
    
    assert(all(all(abs(double(stanf.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');

%% Part B - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 10 random sets of joiny
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(2,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(3,2) - qlim(6,1)) * rand()];
    
    % Calculate the Jacobian in the body frame
     J_b = jacobe_body(S_body,q); 
    
    if plotOn
        stanf.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)]; % swap the rotation and translation components
    assert(all(all(abs(double(stanf.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

%% Part C - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Set the current joint variables
currentQ = zeros(1,6);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba_body(S_body,M,currentQ);

% Generate path to follow
t = linspace(0, 2*pi, nTests);
x = 0.25 * cos(t);
y = 0.25 * sin(t);
z = 0.2 * ones(1,nTests);
path = [x; y; z];

if plotOn
    stanf.teach(currentQ);
    h = plot_ellipse(J_a*J_a');
    title('Inverse Kinematics Test');
    hold on
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end
     
% Iterate over the target points
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S_body, M, currentQ, 'body');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        % YOUR INVERSE KINEMATICS CODE HERE
        % Necessary variables:
        % Current Robot Pose -> currentPose
        % Target Robot Pose ->  targetPose
        % Current Joint Variables -> currentQ
        J=jacoba_body(S_body,M,currentQ);
        deltaQ = pinv(J)*(targetPose-currentPose) ;
        currentQ = currentQ + deltaQ;
        T = fkine(S_body,M,currentQ,'body');
        currentPose = T(1:3,4);
       
        if plotOn
            try
                stanf.teach(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end
end
    

fprintf('\nTest passed successfully.\n');
