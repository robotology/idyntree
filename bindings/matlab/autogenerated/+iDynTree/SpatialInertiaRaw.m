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
        tmp = iDynTreeMEX(636, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = fromRotationalInertiaWrtCenterOfMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(637, self, varargin{:});
    end
    function varargout = getMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(638, self, varargin{:});
    end
    function varargout = getCenterOfMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(639, self, varargin{:});
    end
    function varargout = getRotationalInertiaWrtFrameOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(640, self, varargin{:});
    end
    function varargout = getRotationalInertiaWrtCenterOfMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(641, self, varargin{:});
    end
    function varargout = multiply(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(643, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(644, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(645, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(642, varargin{:});
    end
  end
end
