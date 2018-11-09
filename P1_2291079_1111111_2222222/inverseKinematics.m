%%-----------------------------------------------------------------
%%-- The method inverseKinematics computes the joint configurations
%%   suitable to reach the requested position and orientation in
%%   both elbow configurations.
%% 
%%   Input values are the endeffector position, the endeffector
%%   orientation with the yaw rotation about the z_0-axis and
%%   pitch angle between the z_0-axis and the endeffector direction
%%   described by x_n-axis.
%%-----------------------------------------------------------------
%%-- INPUT
%%   request_position       [3x1]  Requested endeffector position as [x, y, z]'
%%   py                     [2x1]  Requested pitch and yaw as [pitch, yaw]'
%%   l                      [Nx1]  Link lengths
%%
%%-- OUTPUT
%%   qs                     [2xN]  Resulting joint configurations or empty [] if
%%                                 no solution can be found
%%-----------------------------------------------------------------
function qs = inverseKinematics(request_position, py, l)
  % initialize return value
  qs = [];

  % | Implement your code here |
  % v                          v
  qs(:,1) = [py(2) py(2)]';
  xw = request_position(1) - l(4)*sin(py(1));
  yw = request_position(3) - l(1) - l(4)*cos(py(1));
  cosq = (xw^2 + yw^2 - l(3)^2 - l(2)^2)/(2*l(3)*l(2));
  sinq = sqrt(1 - cosq^2);
  positive = 1;
  for i = 1:2
    sinq = positive*sinq;
    qs(i,3) = atan2(sinq,cosq);
    qs(i,2) = atan2(yw,xw) - atan2(l(3)*sinq,l(2) + l(3)*cosq) - pi/2;
    qs(i,4) = -py(1) - qs(i,2) - qs(i,3);   
    positive -= 2;
  end
  
  % ^                          ^
  % | -------- End ----------- |
end
