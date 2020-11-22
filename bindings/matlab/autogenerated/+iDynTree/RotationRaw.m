classdef RotationRaw < iDynTree.Matrix3x3
  methods
    function self = RotationRaw(varargin)
      self@iDynTree.Matrix3x3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(618, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(619, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(620, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(623, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(629, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(630, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(631, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(621, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(622, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(624, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(625, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(626, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(627, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(628, varargin{:});
    end
  end
end
