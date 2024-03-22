classdef AttitudeQuaternionEKFParameters < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = time_step_in_seconds(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1807, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1808, self, varargin{1});
      end
    end
    function varargout = bias_correlation_time_factor(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1809, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1810, self, varargin{1});
      end
    end
    function varargout = accelerometer_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1811, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1812, self, varargin{1});
      end
    end
    function varargout = magnetometer_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1813, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1814, self, varargin{1});
      end
    end
    function varargout = gyroscope_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1815, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1816, self, varargin{1});
      end
    end
    function varargout = gyro_bias_noise_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1817, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1818, self, varargin{1});
      end
    end
    function varargout = initial_orientation_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1819, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1820, self, varargin{1});
      end
    end
    function varargout = initial_ang_vel_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1821, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1822, self, varargin{1});
      end
    end
    function varargout = initial_gyro_bias_error_variance(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1823, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1824, self, varargin{1});
      end
    end
    function varargout = use_magnetometer_measurements(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1825, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1826, self, varargin{1});
      end
    end
    function self = AttitudeQuaternionEKFParameters(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1827, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1828, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
