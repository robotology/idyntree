classdef AttitudeEstimatorState < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = m_orientation(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1648, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1649, self, varargin{1});
      end
    end
    function varargout = m_angular_velocity(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1650, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1651, self, varargin{1});
      end
    end
    function varargout = m_gyroscope_bias(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1652, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1653, self, varargin{1});
      end
    end
    function self = AttitudeEstimatorState(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1654, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1655, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
