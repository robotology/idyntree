classdef Rotation < iDynTree.RotationRaw
  methods
    function self = Rotation(varargin)
      self@iDynTree.RotationRaw(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(632, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(633, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(634, self, varargin{:});
    end
    function varargout = changeCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(635, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(638, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(639, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(640, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(641, self, varargin{:});
    end
    function varargout = fromQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(642, self, varargin{:});
    end
    function varargout = getRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(643, self, varargin{:});
    end
    function varargout = asRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(644, self, varargin{:});
    end
    function varargout = getQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(645, self, varargin{:});
    end
    function varargout = asQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(646, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(663, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(664, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(665, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(636, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(637, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(647, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(648, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(649, varargin{:});
    end
    function varargout = RotAxis(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(650, varargin{:});
    end
    function varargout = RotAxisDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(651, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(652, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(653, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivativeRateOfChange(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(654, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivativeInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(655, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivativeInverseRateOfChange(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(656, varargin{:});
    end
    function varargout = QuaternionRightTrivializedDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(657, varargin{:});
    end
    function varargout = QuaternionRightTrivializedDerivativeInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(658, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(659, varargin{:});
    end
    function varargout = RotationFromQuaternion(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(660, varargin{:});
    end
    function varargout = leftJacobian(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(661, varargin{:});
    end
    function varargout = leftJacobianInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(662, varargin{:});
    end
  end
end
