classdef Rotation < iDynTree.RotationRaw
  methods
    function self = Rotation(varargin)
      self@iDynTree.RotationRaw(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(538, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(539, self, varargin{:});
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(540, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(541, self, varargin{:});
    end
    function varargout = changeCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(542, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(545, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(546, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(547, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(548, self, varargin{:});
    end
    function varargout = getRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(549, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(557, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(558, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(559, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(543, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(544, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(550, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(551, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(552, varargin{:});
    end
    function varargout = RotAxis(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(553, varargin{:});
    end
    function varargout = RotAxisDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(554, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(555, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(556, varargin{:});
    end
  end
end
