classdef Rotation < iDynTree.RotationRaw
  methods
    function self = Rotation(varargin)
      self@iDynTree.RotationRaw(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(496, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(497, self, varargin{:});
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(498, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(499, self, varargin{:});
    end
    function varargout = changeCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(500, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(503, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(504, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(505, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(506, self, varargin{:});
    end
    function varargout = getRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(507, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(515, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(516, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(517, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(501, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(502, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(508, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(509, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(510, varargin{:});
    end
    function varargout = RotAxis(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(511, varargin{:});
    end
    function varargout = RotAxisDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(512, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(513, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(514, varargin{:});
    end
  end
end
