function uOpt = optCtrl(obj, ~, y, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

if ~iscell(y)
    y = num2cell(y);
end

det = deriv{1}.*cos(y{3}) + deriv{2}.*sin(y{3});

%% Optimal control
if strcmp(uMode, 'max')
  uOpt{1} = (det>=0)*obj.speed_bounds(2) + (det<=0)*obj.speed_bounds(1);
  uOpt{2} = (deriv{obj.dims==3}>=0)*obj.wMax + (deriv{obj.dims==3}<0)*(-obj.wMax);
elseif strcmp(uMode, 'min')
  uOpt{1} = (det>=0)*obj.speed_bounds(1) + (det<=0)*obj.speed_bounds(2);
  uOpt{2} = (deriv{obj.dims==3}>=0)*(-obj.wMax) + (deriv{obj.dims==3}<0)*obj.wMax;
else
  error('Unknown uMode!')
end

end