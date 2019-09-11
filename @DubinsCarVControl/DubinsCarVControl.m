classdef DubinsCarVControl < DynSys
  properties
    % Angular control bounds
    wMax
    
    speed_bounds % Constant speed
    
    % Disturbance
    dMax
    
    % Dimensions that are active
    dims
  end
  
  methods
    function obj = DubinsCarVControl(x, wMax, speed_bounds, dMax, dims)
      % obj = DubinsCar(x, wMax, speed, dMax, dims)
      %     Dubins Car class
      %
      % Dynamics:
      %    \dot{x}_1 = v * cos(x_3) + d1
      %    \dot{x}_2 = v * sin(x_3) + d2
      %    \dot{x}_3 = u            + d3
      %         v \in [vrange(1), vrange(2)]
      %         u \in [-wMax, wMax]
      %         d \in [-dMax, dMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos; theta]
      %   wMax   - maximum turn rate
      %   vrange - speed range
      %   dMax   - disturbance bounds
      %
      % Output:
      %   obj       - a DubinsCar object
      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      if nargin < 2
        wMax = 1;
      end
      
      if nargin < 3
        speed_bounds = [0, 5];
      end
      
      if nargin < 4
        dMax = [0; 0; 0];
      end
      
      if nargin < 5
        dims = 1:3;
      end
      
      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      obj.hdim = find(dims == 3);   % Heading dimensions
      obj.nx = length(dims);
      obj.nu = 2;
      obj.nd = 3;
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.wMax = wMax;
      obj.speed_bounds = speed_bounds;
      obj.dMax = dMax;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
