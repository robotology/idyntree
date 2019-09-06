classdef AttitudeQuaternionEKFParameters < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = time_step_in_seconds(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1721, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1722, self, varargin{1});
      end
    end
    function varargout = bias_correlation_time_factor(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1723, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1724, self, varargin{1});
      end
    end
    function varargout = accelerometer_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1725, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1726, self, varargin{1});
      end
    end
    function varargout = magnetometer_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1727, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1728, self, varargin{1});
      end
    end
    function varargout = gyroscope_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1729, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1730, self, varargin{1});
      end
    end
    function varargout = gyro_bias_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1731, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1732, self, varargin{1});
      end
    end
    function varargout = initial_orientation_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1733, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1734, self, varargin{1});
      end
    end
    function varargout = initial_ang_vel_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1735, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1736, self, varargin{1});
      end
    end
    function varargout = initial_gyro_bias_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1737, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1738, self, varargin{1});
      end
    end
    function varargout = use_magnetometer_measurements(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1739, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1740, self, varargin{1});
      end
    end
    function self = AttitudeQuaternionEKFParameters(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1741, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1742, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
