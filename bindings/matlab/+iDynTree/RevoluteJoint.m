classdef RevoluteJoint < iDynTree.MovableJointImpl1
  methods
    function self = RevoluteJoint(varargin)
      self@iDynTree.MovableJointImpl1(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(802, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(803, self);
        self.swigPtr=[];
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(804, self, varargin{:});
    end
    function varargout = setAttachedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(805, self, varargin{:});
    end
    function varargout = setRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(806, self, varargin{:});
    end
    function varargout = setAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(807, self, varargin{:});
    end
    function varargout = getFirstAttachedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(808, self, varargin{:});
    end
    function varargout = getSecondAttachedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(809, self, varargin{:});
    end
    function varargout = getAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(810, self, varargin{:});
    end
    function varargout = getRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(811, self, varargin{:});
    end
    function varargout = getTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(812, self, varargin{:});
    end
    function varargout = getTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(813, self, varargin{:});
    end
    function varargout = getMotionSubspaceVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(814, self, varargin{:});
    end
    function varargout = computeChildPosVelAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(815, self, varargin{:});
    end
    function varargout = computeChildVelAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(816, self, varargin{:});
    end
    function varargout = computeJointTorque(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(817, self, varargin{:});
    end
  end
  methods(Static)
  end
end
