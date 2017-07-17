% SLAM2D A 2D EKF−SLAM algorithm with simulation and graphics.
%
% HELP NOTES:
% 1. The robot state is defined by [xr;yr;ar] with [xr;yr] the position
% and [ar] the orientation angle in the plane.
% 2. The landmark states are simply Li=[xi;yi]. There are a number of N
% landmarks organized in a 2−by−N matrix W=[L1 L2 ... Ln]
% so that Li = W(:,i).
% 3. The control signal for the robot is U=[dx;da] where [dx] is a forward
% motion and [da] is the angle of rotation.
% 4. The motion perturbation is additive Gaussian noise n=[nx;na] with
% covariance Q, which adds to the control signal.
% 5. The measurements are range−and−bearing Yi=[di;ai], with [di] the
% distance from the robot to landmark Li, and [ai] the bearing angle from
% the robot's x−axis.
% 6. The simulated variables are written in capital letters,
% R: robot
% W: set of landmarks or 'world'
% Y: set of landmark measurements Y=[Y1 Y2 ... YN]
% 7. The true map is [xr;yr;ar;x1;y1;x2;y2;x3;y3; ... ;xN;yN]
% 8. The estimated map is Gaussian, defined by
% x: mean of the map
% P: covariances matrix of the map
% 9. The estimated entities (robot and landmarks) are extracted from {x,P}
% via pointers, denoted in small letters as follows:
% r: pointer to robot state. r=[1,2,3]
% l: pointer to landmark i. We have for example l=[4,5] if i=1,
% l=[6,7] if i=2, and so on.
% m: pointers to all used landmarks.
% rl: pointers to robot and one landmark.
% rm: pointers to robot and all landmarks (the currently used map).
% Therefore: x(r) is the robot state,
% x(l) is the state of landmark i
% P(r,r) is the covariance of the robot
% P(l,l) is the covariance of landmark i
% P(r,l) is the cross−variance between robot and lmk i
% P(rm,rm) is the current full covariance −− the rest is
% unused.
% NOTE: Pointers are always row−vectors of integers.
% 10. Managing the map space is done through the variable mapspace.
% mapspace is a logical vector the size of x. If mapspace(i) = false,
% then location i is free. Oterwise mapspace(i) = true. Use it as
% follows:
% * query for n free spaces: s = find(mapspace==false, n);
% * block positions indicated in vector s: mapspace(s) = true;
% * liberate positions indicated in vector s: mapspace(s) = false;
% 11. Managing the existing landmarks is done through the variable landmarks.
% landmarks is a 2−by−N matrix of integers. l=landmarks(:,i) are the
% pointers of landmark i in the state vector x, so that x(l) is the
% state of landmark i. Use it as follows:
% * query 1 free space for a new landmark: i = find(landmarks(1,:)==0,1)
% * associate indices in vector s to landmark i: landmarks(:,i) = s
% * liberate landmark i: landmarks(:,i) = 0;
% 12. Graphics objects are Matlab 'handles'. See Matlab doc for information.
% 13. Graphic objects include:
% RG: simulated robot
% WG: simulated set of landmarks
% rG: estimated robot
% reG: estimated robot ellipse
% lG: estimated landmarks
% leG: estimated landmark ellipses
% (c) 2010, 2011, 2012 Joan Sola.

% I. INITIALIZE
% W: set of external landmarks
% 
W
%
N
%
R
%
U
%
Y I.1 SIMULATOR −− use capital letters for variable names

= cloister(−4,4,−4,4,7);
% Type 'help cloister' for help
N: number of landmarks
= size(W,2);
R: robot pose [x ; y ; alpha]
= [0;−2;0];
U: control [d x ; d alpha]
= [0.1 ; 0.05]; % fixing advance and turn increments creates a circle
Y: measurements of all landmarks
= zeros(2, N);
%
%
%
x
%
P I.2 ESTIMATOR
Map: Gaussian {x,P}
x: state vector's mean
= zeros(numel(R)+numel(W), 1);
P: state vector's covariances matrix
= zeros(numel(x),numel(x));
% System noise: Gaussian {0,Q}
q = [.01;.02];
% amplitude or standard deviation
Q = diag(q.ˆ2);
% covariances matrix
% Measurement noise: Gaussian {0,S}
s = [.1;1*pi/180]; % amplitude or standard deviation
S = diag(s.ˆ2);
% covariances matrix
% Map management
mapspace = false(1,numel(x)); % See Help Note #10 above.
% Landmarks management
landmarks = zeros(2, N); % See Help Note #11 above
% Place robot
r
=
mapspace(r) =
x(r)
=
P(r,r)
=
in map
find(mapspace==false, numel(R) ); % set robot pointer
true; % block map positions
R;
% initialize robot states
0;
% initialize robot covariance
% I.3 GRAPHICS −− use the variable names of simulated and estimated