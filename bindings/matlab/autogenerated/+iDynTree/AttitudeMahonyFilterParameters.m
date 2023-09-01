classdef AttitudeMahonyFilterParameters < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = time_step_in_seconds(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1698, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1699, self, varargin{1});
      end
    end
    function varargout = kp(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1700, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1701, self, varargin{1});
      end
    end
    function varargout = ki(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1702, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1703, self, varargin{1});
      end
    end
    function varargout = use_magnetometer_measurements(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1704, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1705, self, varargin{1});
      end
    end
    function varargout = confidence_magnetometer_measurements(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1706, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1707, self, varargin{1});
      end
    end
    function self = AttitudeMahonyFilterParameters(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1708, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1709, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
