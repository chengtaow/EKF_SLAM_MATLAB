function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% Compute the new mu based on the noise-free (odometry-based) motion model
% use the function normalize_angle to normalize theta after the update
omiga = normalize_angle(u.r2 - u.r1);
theta = mu(3);
mu(1) = mu(1) - (u.t/omiga)*sin(mu(3)) + (u.t/omiga)*sin(mu(3)+omiga);
mu(2) = mu(2) + (u.t/omiga)*cos(mu(3)) - (u.t/omiga)*cos(mu(3)+omiga);
mu(3) = normalize_angle(mu(3) - omiga);
% Compute the 3x3 Jacobian Gx of the motion model
Gx = eye(3);
Gx(1,3) = - u.t/omiga*sin(theta) + u.t/omiga*sin(theta+omiga);
Gx(2,3) = u.t/omiga*cos(theta) - u.t/omiga*cos(theta+omiga);

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
 
% Compute the predicted sigma after incorporating the motion
robsigma = Gx*sigma(1:3,1:3)*Gx.' + R3;
rmsigma = Gx*sigma(1:3,4:size(sigma,1));
mapsigma = sigma(4:size(sigma,1),4:size(sigma),1);

sigma = [[robsigma rmsigma];[rmsigma.' mapsigma]];

end
