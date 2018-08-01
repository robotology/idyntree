classdef SpatialInertiaRaw < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SpatialInertiaRaw(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(649, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = fromRotationalInertiaWrtCenterOfMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(650, self, varargin{:});
    end
    function varargout = getMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(651, self, varargin{:});
    end
    function varargout = getCenterOfMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(652, self, varargin{:});
    end
    function varargout = getRotationalInertiaWrtFrameOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(653, self, varargin{:});
    end
    function varargout = getRotationalInertiaWrtCenterOfMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(654, self, varargin{:});
    end
    function varargout = multiply(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(656, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(657, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(658, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(655, varargin{:});
    end
  end
end
