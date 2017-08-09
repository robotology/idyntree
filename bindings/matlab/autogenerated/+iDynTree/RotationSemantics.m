classdef RotationSemantics < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = RotationSemantics(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(710, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setToUnknown(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(711, self, varargin{:});
    end
    function varargout = getOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(712, self, varargin{:});
    end
    function varargout = getBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(713, self, varargin{:});
    end
    function varargout = getReferenceOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(714, self, varargin{:});
    end
    function varargout = getRefBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(715, self, varargin{:});
    end
    function varargout = getCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(716, self, varargin{:});
    end
    function varargout = setOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(717, self, varargin{:});
    end
    function varargout = setBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(718, self, varargin{:});
    end
    function varargout = setReferenceOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(719, self, varargin{:});
    end
    function varargout = setRefBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(720, self, varargin{:});
    end
    function varargout = setCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(721, self, varargin{:});
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(722, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(723, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(724, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(727, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(728, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(729, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(725, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(726, varargin{:});
    end
  end
end
