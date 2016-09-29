classdef GeomVector3Semantics__LinearMotionVector3Semantics < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = GeomVector3Semantics__LinearMotionVector3Semantics(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(323, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setToUnknown(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(324, self, varargin{:});
    end
    function varargout = getBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(325, self, varargin{:});
    end
    function varargout = getRefBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(326, self, varargin{:});
    end
    function varargout = getCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(327, self, varargin{:});
    end
    function varargout = isUnknown(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(328, self, varargin{:});
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(329, self, varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(332, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(333, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(330, varargin{:});
    end
    function varargout = inverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(331, varargin{:});
    end
  end
end
