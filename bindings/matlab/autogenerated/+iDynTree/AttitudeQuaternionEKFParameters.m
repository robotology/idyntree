classdef AttitudeQuaternionEKFParameters < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = time_step_in_seconds(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1694, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1695, self, varargin{1});
      end
    end
    function varargout = bias_correlation_time_factor(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1696, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1697, self, varargin{1});
      end
    end
    function varargout = accelerometer_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1698, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1699, self, varargin{1});
      end
    end
    function varargout = magnetometer_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1700, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1701, self, varargin{1});
      end
    end
    function varargout = gyroscope_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1702, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1703, self, varargin{1});
      end
    end
    function varargout = gyro_bias_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1704, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1705, self, varargin{1});
      end
    end
    function varargout = initial_orientation_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1706, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1707, self, varargin{1});
      end
    end
    function varargout = initial_ang_vel_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1708, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1709, self, varargin{1});
      end
    end
    function varargout = initial_gyro_bias_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1710, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1711, self, varargin{1});
      end
    end
    function varargout = use_magnetometer_measurements(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1712, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1713, self, varargin{1});
      end
    end
    function self = AttitudeQuaternionEKFParameters(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1714, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1715, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
