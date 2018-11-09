%%-----------------------------------------------------------------
%%-- The method forwardKinematicsDH computes the transformation
%%   matrices, which describe the position and orientation of the
%%   joints of the manipulator with regard to the base coordinate
%%   system.
%%   Input values are the struct containing the DH-Parameter of
%%   all links of the kinematic chain and the joint variables.
%%-----------------------------------------------------------------
%%-- INPUT
%%   dh       [1xN]  Data structure containing four DH-Parameter and one joint type
%%    .theta  [1x1]  DH-Parameter theta_i
%%    .d      [1x1]  DH-Parameter d_i
%%    .a      [1x1]  DH-Parameter a_i
%%    .alpha  [1x1]  DH-Parameter alpha_i
%%    .rho    [1x1]  Joint type (0 = linear, 1 = rotary)
%%   q        [Nx1]  Joint angles in rad
%%
%%-- OUTPUT
%%   TM      [1xN]  Data structure of all transformation matrices
%%    .T     [4x4]  Transformation matrix of the i-th robot link with regard to the base
%%-----------------------------------------------------------------
function TM = forwardKinematicsDH(dh, q)
  % initialize return value
  TM = repmat(struct('T',eye(4,4)), length(dh), 1 );

  % | Implement your code here |
  % v                          v
  TM(1).T = computeTransformationDH(dh(1), q(1));
  for i = 2 : length(dh)
    TM(i).T = TM(i-1).T* computeTransformationDH(dh(i), q(i));
  end
  % ^                          ^
  % | -------- End ----------- |
end
