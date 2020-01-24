classdef RevoluteJoint < iDynTree.MovableJointImpl1
  methods
    function self = RevoluteJoint(varargin)
      self@iDynTree.MovableJointImpl1(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1032, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1033, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1034, self, varargin{:});
    end
    function varargout = setAttachedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1035, self, varargin{:});
    end
    function varargout = setRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1036, self, varargin{:});
    end
    function varargout = setAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1037, self, varargin{:});
    end
    function varargout = getFirstAttachedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1038, self, varargin{:});
    end
    function varargout = getSecondAttachedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1039, self, varargin{:});
    end
    function varargout = getAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1040, self, varargin{:});
    end
    function varargout = getRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1041, self, varargin{:});
    end
    function varargout = getTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1042, self, varargin{:});
    end
    function varargout = getTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1043, self, varargin{:});
    end
    function varargout = getMotionSubspaceVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1044, self, varargin{:});
    end
    function varargout = computeChildPosVelAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1045, self, varargin{:});
    end
    function varargout = computeChildVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1046, self, varargin{:});
    end
    function varargout = computeChildVelAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1047, self, varargin{:});
    end
    function varargout = computeChildAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1048, self, varargin{:});
    end
    function varargout = computeChildBiasAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1049, self, varargin{:});
    end
    function varargout = computeJointTorque(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1050, self, varargin{:});
    end
    function varargout = hasPosLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1051, self, varargin{:});
    end
    function varargout = enablePosLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1052, self, varargin{:});
    end
    function varargout = getPosLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1053, self, varargin{:});
    end
    function varargout = getMinPosLimit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1054, self, varargin{:});
    end
    function varargout = getMaxPosLimit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1055, self, varargin{:});
    end
    function varargout = setPosLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1056, self, varargin{:});
    end
  end
  methods(Static)
  end
end
