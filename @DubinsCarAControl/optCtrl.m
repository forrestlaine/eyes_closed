function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

angular_rate_bound = min(deriv{obj.dims==3}/obj.rMin, obj.wMax);

%% Optimal control
if strcmp(uMode, 'max')
  uOpt{1} = (deriv{obj.dims==3}>=0)*obj.accel_bounds(2) + (deriv{obj.dims==3}<0)*obj.accel_bounds(1);
  uOpt{2} = (deriv{obj.dims==4}>=0).*angular_rate_bound + (deriv{obj.dims==4}<0).*(-angular_rate_bound);
elseif strcmp(uMode, 'min')
  uOpt{1} = (deriv{obj.dims==3}>=0)*obj.accel_bounds(1) + (deriv{obj.dims==3}<0)*obj.accel_bounds(2);
  uOpt{2} = (deriv{obj.dims==4}>=0).*(-angular_rate_bound) + (deriv{obj.dims==4}<0).*angular_rate_bound;
else
  error('Unknown uMode!')
end

end