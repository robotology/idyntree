classdef RevoluteJoint < iDynTree.MovableJointImpl1
  methods
    function self = RevoluteJoint(varargin)
      self@iDynTree.MovableJointImpl1(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(859, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(860, self);
        self.swigPtr=[];
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(861, self, varargin{:});
    end
    function varargout = setAttachedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(862, self, varargin{:});
    end
    function varargout = setRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(863, self, varargin{:});
    end
    function varargout = setAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(864, self, varargin{:});
    end
    function varargout = getFirstAttachedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(865, self, varargin{:});
    end
    function varargout = getSecondAttachedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(866, self, varargin{:});
    end
    function varargout = getAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(867, self, varargin{:});
    end
    function varargout = getRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(868, self, varargin{:});
    end
    function varargout = getTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(869, self, varargin{:});
    end
    function varargout = getTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(870, self, varargin{:});
    end
    function varargout = getMotionSubspaceVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(871, self, varargin{:});
    end
    function varargout = computeChildPosVelAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(872, self, varargin{:});
    end
    function varargout = computeChildVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(873, self, varargin{:});
    end
    function varargout = computeChildVelAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(874, self, varargin{:});
    end
    function varargout = computeJointTorque(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(875, self, varargin{:});
    end
  end
  methods(Static)
  end
end
