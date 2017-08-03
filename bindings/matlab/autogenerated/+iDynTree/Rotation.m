classdef Rotation < iDynTree.RotationRaw
  methods
    function self = Rotation(varargin)
      self@iDynTree.RotationRaw(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(725, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(726, self, varargin{:});
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(727, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(728, self, varargin{:});
    end
    function varargout = changeCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(729, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(732, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(733, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(734, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(735, self, varargin{:});
    end
    function varargout = fromQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(736, self, varargin{:});
    end
    function varargout = getRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(737, self, varargin{:});
    end
    function varargout = asRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(738, self, varargin{:});
    end
    function varargout = getQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(739, self, varargin{:});
    end
    function varargout = asQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(740, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(753, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(754, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(755, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(730, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(731, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(741, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(742, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(743, varargin{:});
    end
    function varargout = RotAxis(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(744, varargin{:});
    end
    function varargout = RotAxisDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(745, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(746, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(747, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivativeInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(748, varargin{:});
    end
    function varargout = QuaternionRightTrivializedDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(749, varargin{:});
    end
    function varargout = QuaternionRightTrivializedDerivativeInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(750, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(751, varargin{:});
    end
    function varargout = RotationFromQuaternion(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(752, varargin{:});
    end
  end
end
