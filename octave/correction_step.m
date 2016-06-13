function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2);
N = size(observedLandmarks, 2);
% Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
% ExpectedZ: vectorized form of all expected measurements in the same form.
% They are initialized here and should be filled out in the for loop below
Z = zeros(2*m, 1);
%expectedZ = zeros(m*2, 1);

% Iterate over the measurements and compute the H matrix
% (stacked Jacobian blocks of the measurement function)
% H will be 2m x 2N+3
H = zeros(2*m, 2*N+3);
h = zeros(2*m, 1);
for i = 1:m
	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;
    if isempty(z(i).id) 
        continue
    end
    %disp(i)
    Z(2*i-1) = z(i).range;
    Z(2*i) = z(i).bearing;
	% If the landmark is obeserved for the first time:
	if observedLandmarks(landmarkId) == false
		% TODO: Initialize its pose in mu based on the measurement and the current robot pose:
		mu(2+landmarkId*2) = mu(1) + z(i).range * cos(mu(3)+z(i).bearing);
        mu(3+landmarkId*2) = mu(2) + z(i).range * sin(mu(3)+z(i).bearing);
		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
    end
    
    delta_x = mu(2+i*2)-mu(1);
    delta_y = mu(3+i*2)-mu(2);
    delta = [delta_x; delta_y];
    q = delta.'*delta;
    h(2*i-1) = sqrt(q);
    h(2*i) = atan2(delta_y, delta_x) - mu(3);
    H(2*i-1,1) = -sqrt(q)/q*delta_x;
    H(2*i-1,2) = -sqrt(q)/q*delta_y;
    H(2*i-1,3) = 0;
    H(2*i-1,2*i+2) = sqrt(q)/q*delta_x;
    H(2*i-1,2*i+3) = sqrt(q)/q*delta_y;
    H(2*i,1) = delta_y/q;
    H(2*i,2) = -delta_x/q;
    H(2*i,3) = -1;
    H(2*i,2*i+2) = -delta_y/q;
    H(2*i,2*i+3) = delta_x/q;
	% TODO: Add the landmark measurement to the Z vector
	
	% TODO: Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:

	% TODO: Compute the Jacobian Hi of the measurement function h for this observation
	
	% Augment H with the new Hi
	%H = [H;Hi];	
end

% TODO: Construct the sensor noise matrix Q
Q = zeros(2*m,2*m);
sen_noise = 0.01;
for j = 1:2*m
    Q(j,j) = sen_noise;
end
% TODO: Compute the Kalman gain
K = sigma * H.' / (H * sigma * H.' + Q);
% TODO: Compute the difference between the expected and recorded measurements.
subZ = normalize_all_bearings(Z - h);
mu = mu + K * (subZ);
mu(3) = normalize_angle(mu(3));
% Remember to normalize the bearings after subtracting!
% (hint: use the normalize_all_bearings function available in tools)
sigma = (eye(2*N+3,2*N+3) - K * H) * sigma;
% TODO: Finish the correction step by computing the new mu and sigma.
% Normalize theta in the robot pose.

end
