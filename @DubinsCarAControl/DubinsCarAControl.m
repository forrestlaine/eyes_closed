classdef DubinsCarAControl < DynSys
  properties
    % min turning radius
    rMin
    
    % max angular velocity
    wMax
    
    % bounds on acceleration inputs
    accel_bounds 
    
    % Disturbance
    dMax
    
    % Dimensions that are active
    dims
  end
  
  methods
    function obj = DubinsCarAControl(x, rMin, wMax, accel_bounds, dMax, dims)
      % obj = DubinsCar(x, rMin, wMax, accel_bounds, dMax, dims)
      %     Dubins Car class
      %
      % Dynamics:
      %    \dot{x}_1 = x_3 * cos(x_4) + d1
      %    \dot{x}_2 = x_3 * sin(x_4) + d2
      %    \dot{x}_3 = u1             + d3
      %    \dot{x}_4 = u2             + d4
      %         u1 \in [accel_bounds(1), accel_bounds(2)]
      %         u2 \in [max(-x4/rMin, -wMax), min(x4/rMin, wMax)]
      %         d \in [-dMax, dMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos; vel; theta]
      %   wMax   - maximum turn rate
      %   accel_range - acceleration range
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
        rMin = 5.0;
      end
      
      if nargin < 3
        wMax = 1;
      end
      
      if nargin < 4
        accel_bounds = [-1, 1];
      end
      
      if nargin < 5
        dMax = [0; 0; 0; 0];
      end
      
      if nargin < 6
        dims = 1:4;
      end
      
      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      obj.hdim = find(dims == 4);   % Heading dimensions
      obj.nx = length(dims);
      obj.nu = 2;
      obj.nd = 3;
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.wMax = wMax;
      obj.rMin = rMin;
      obj.accel_bounds = accel_bounds;
      obj.dMax = dMax;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
