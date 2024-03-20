classdef Rotation < iDynTree.Matrix3x3
  methods
    function self = Rotation(varargin)
      self@iDynTree.Matrix3x3(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(681, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(682, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(683, self, varargin{:});
    end
    function varargout = changeCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(684, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(687, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(688, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(689, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(690, self, varargin{:});
    end
    function varargout = fromQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(691, self, varargin{:});
    end
    function varargout = getRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(692, self, varargin{:});
    end
    function varargout = asRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(693, self, varargin{:});
    end
    function varargout = getQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(694, self, varargin{:});
    end
    function varargout = asQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(695, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(712, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(713, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(714, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(685, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(686, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(696, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(697, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(698, varargin{:});
    end
    function varargout = RotAxis(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(699, varargin{:});
    end
    function varargout = RotAxisDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(700, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(701, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(702, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivativeRateOfChange(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(703, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivativeInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(704, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivativeInverseRateOfChange(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(705, varargin{:});
    end
    function varargout = QuaternionRightTrivializedDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(706, varargin{:});
    end
    function varargout = QuaternionRightTrivializedDerivativeInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(707, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(708, varargin{:});
    end
    function varargout = RotationFromQuaternion(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(709, varargin{:});
    end
    function varargout = leftJacobian(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(710, varargin{:});
    end
    function varargout = leftJacobianInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(711, varargin{:});
    end
  end
end
