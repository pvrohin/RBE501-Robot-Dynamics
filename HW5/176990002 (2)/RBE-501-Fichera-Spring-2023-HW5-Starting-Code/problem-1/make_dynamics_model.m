function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link


%% Create the manipulator
L1 = 0.3; % Lenght of Link 1 [m]
L2 = 0.3; % Lenght of Link 2 [m]
L3 = 0.15; % Lenght of Link 3 [m]
w  = 0.04; % Link Width [m]
l  = 0.04; % Link Depth [m]

R1 = eye(3);
R2 = [1 0 0; 0 0 1; 0 -1 0];
R3 = [1 0 0; 0 0 1; 0 -1 0];
R4 = [0 1 0; 0 0 1; 1 0 0];
p1 = [0 0 L1/2]';
p2 = [0 L2/2 L1/2]';
p3 = [0 L3/2 L2/2]';
p4 = [0 0 (L1-L3)/2]';


% Link poses when the robot is in the home configuration
M01 = [R1 p1; 0 0 0 1];
M12 = [R2 p2; 0 0 0 1];
M23 = [R3 p3; 0 0 0 1];
M34 = [R4 p4; 0 0 0 1];

Mlist = cat(3, M01, M12, M23, M34);

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
m1 = 5;   % Mass of Link 1 [kg]
m2 = 1;   % Mass of Link 2 [kg]
m3 = 1;   % Mass of Link 3 [kg]

%O = [0 0 0;0 0 0;0 0 0];

% Spatial Inertia Matrices
% G1 = [O O; O m1*eye(3)];
% G2 = [O O; O m2*eye(3)];
% G3 = [O O; O m3*eye(3)];

h1=0.3;
Ixx1=(m1*(w*w+h1*h1))/12;
Iyy1=(m1*(l*l+h1*h1))/12;
Izz1=(m1*(l*l+w*w))/12;
a1=[Ixx1 0 0;
    0 Iyy1 0;
    0 0 Izz1];
G1 = [a1 zeros(3,3);
      zeros(3,3) m1*eye(3)]; %Spatial Inertia Matrix for link 1

h2=0.3;
Ixx2=(m2*(w*w+h2*h2))/12;
Iyy2=(m2*(l*l+h2*h2))/12;
Izz2=(m2*(l*l+w*w))/12;
a2=[Ixx2 0 0;
    0 Iyy2 0;
    0 0 Izz2];
G2 = [a2 zeros(3,3);
      zeros(3,3) m2*eye(3)]; %Spatial Inertia Matrix for link 2

h3=0.15;
Ixx3=(m3*(w*w+h3*h3))/12;
Iyy3=(m3*(l*l+h3*h3))/12;
Izz3=(m3*(l*l+w*w))/12;
a3=[Ixx3 0 0;
    0 Iyy3 0;
    0 0 Izz3];
G3 = [a3 zeros(3,3);
      zeros(3,3) m3*eye(3)]; %Spatial Inertia Matrix for link 3

Glist = cat(3, G1, G2, G3);

end