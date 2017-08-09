classdef DynamicsComputations < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = DynamicsComputations(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1664, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1665, self);
        self.swigPtr=[];
      end
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1666, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1667, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1668, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1669, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1670, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1671, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1672, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1673, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1674, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1675, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1676, self, varargin{:});
    end
    function varargout = getWorldBaseTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1677, self, varargin{:});
    end
    function varargout = getBaseTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1678, self, varargin{:});
    end
    function varargout = getJointPos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1679, self, varargin{:});
    end
    function varargout = getJointVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1680, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1681, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1682, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1683, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1684, self, varargin{:});
    end
    function varargout = getFrameTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1685, self, varargin{:});
    end
    function varargout = getFrameTwistInWorldOrient(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1686, self, varargin{:});
    end
    function varargout = getFrameProperSpatialAcceleration(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1687, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1688, self, varargin{:});
    end
    function varargout = getLinkInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1689, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1690, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1691, self, varargin{:});
    end
    function varargout = getJointLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1692, self, varargin{:});
    end
    function varargout = inverseDynamics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1693, self, varargin{:});
    end
    function varargout = getFreeFloatingMassMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1694, self, varargin{:});
    end
    function varargout = getFrameJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1695, self, varargin{:});
    end
    function varargout = getDynamicsRegressor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1696, self, varargin{:});
    end
    function varargout = getModelDynamicsParameters(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1697, self, varargin{:});
    end
    function varargout = getCenterOfMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1698, self, varargin{:});
    end
    function varargout = getCenterOfMassJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1699, self, varargin{:});
    end
  end
  methods(Static)
  end
end
