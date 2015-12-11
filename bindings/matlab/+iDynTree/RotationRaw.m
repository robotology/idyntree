classdef RotationRaw < iDynTree.Matrix3x3
  methods
    function self = RotationRaw(varargin)
      self@iDynTree.Matrix3x3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(464, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(465, self);
        self.swigPtr=[];
      end
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(466, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(467, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(470, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(476, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(477, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(468, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(469, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(471, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(472, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(473, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(474, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(475, varargin{:});
    end
  end
end
