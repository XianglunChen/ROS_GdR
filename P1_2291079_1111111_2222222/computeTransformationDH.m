%%-----------------------------------------------------------------
%%-- The function computeTransformationDH computes, given the structure
%%   dh containing the DH-Parameter as well as the joint value q, the
%%   transformation matrix of one robot link.
%%-----------------------------------------------------------------
%%-- INPUT
%%   dh       [1xN]  Data structure containing four DH-Parameter and one joint type
%%    .theta  [1x1]  DH-Parameter theta_i
%%    .d      [1x1]  DH-Parameter d_i
%%    .a      [1x1]  DH-Parameter a_i
%%    .alpha  [1x1]  DH-Parameter alpha_i
%%    .rho    [1x1]  Joint type (0 = linear, 1 = rotary)
%%   q        [Nx1]  Joint angle in rad
%%
%%-- OUTPUT
%%   T        [4x4]  Transformation matrix of the robot link
%%-----------------------------------------------------------------
function T = computeTransformationDH(dh, q)
  % initialize return value
  T = eye(4);

  % | Implement your code here |
  % v                          v
  theta = dh.theta + q * dh.rho;
  d = dh.d + q * (1 - dh.rho);
  
  Rot_theta = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
  Trans_d = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
  Trans_a = [1 0 0 dh.a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
  Rot_alpha = [1 0 0 0; 0 cos(dh.alpha) -sin(dh.alpha) 0; 0 sin(dh.alpha) cos(dh.alpha) 0; 0 0 0 1];
  
  T = Rot_theta * Trans_d * Trans_a * Rot_alpha;
  % ^                          ^
  % | -------- End ----------- |
end
