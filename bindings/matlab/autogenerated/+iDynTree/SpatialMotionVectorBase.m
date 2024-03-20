classdef SpatialMotionVectorBase < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SpatialMotionVectorBase(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(505, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = getLinearVec3(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(506, self, varargin{:});
    end
    function varargout = getAngularVec3(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(507, self, varargin{:});
    end
    function varargout = setLinearVec3(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(508, self, varargin{:});
    end
    function varargout = setAngularVec3(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(509, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(510, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(511, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(512, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(513, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(514, self, varargin{:});
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(515, self, varargin{:});
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(516, self, varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(519, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(520, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(521, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(522, self, varargin{:});
    end
    function varargout = asVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(524, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(525, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(526, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(527, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(528, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(529, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(517, varargin{:});
    end
    function varargout = inverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(518, varargin{:});
    end
    function varargout = Zero(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(523, varargin{:});
    end
  end
end
