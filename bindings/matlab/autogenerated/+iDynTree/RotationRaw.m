classdef RotationRaw < iDynTree.Matrix3x3
  methods
    function self = RotationRaw(varargin)
      self@iDynTree.Matrix3x3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(741, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(742, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(743, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(746, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(752, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(753, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(754, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(744, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(745, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(747, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(748, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(749, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(750, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(751, varargin{:});
    end
  end
end
