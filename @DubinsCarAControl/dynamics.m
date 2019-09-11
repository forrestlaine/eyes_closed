function dx = dynamics(obj, ~, x, u, d)
% Dynamics of the Dubins Car
%    \dot{x}_1 = x_3 * cos(x_4)
%    \dot{x}_2 = x_3 * sin(x_4)
%    \dot{x}_3 = a
%    \dot{x}_4 = w
%   Control: u1 = a; u2 = w;
%
% Forrest Laine, 2019-07-27

if nargin < 5
  d = [0; 0; 0; 0];
end

if iscell(x)
  dx = cell(length(obj.dims), 1);
  
  for i = 1:length(obj.dims)
    dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  dx(1) = x(3) * cos(x(4)) + d(1);
  dx(2) = x(3) * sin(x(4)) + d(2);
  dx(3) = u(1) + d(3);
  dx(4) = u(2) + d(4);
end
end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)

switch dim
  case 1
    dx = x{dims==3} .* cos(x{dims==4}) + d{1};
  case 2
    dx = x{dims==3} .* sin(x{dims==4}) + d{2};
  case 3
    dx = u{1} + d{3};
  case 4
    dx = u{2} + d{4};
  otherwise
    error('Only dimension 1-3 are defined for dynamics of DubinsCar!')
end
end