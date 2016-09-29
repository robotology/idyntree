classdef RotationRaw < iDynTree.Matrix3x3
  methods
    function self = RotationRaw(varargin)
      self@iDynTree.Matrix3x3(SwigRef.Null);
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
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(637, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(638, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(641, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(647, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(648, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(649, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(639, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(640, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(642, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(643, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(644, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(645, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(646, varargin{:});
    end
  end
end
