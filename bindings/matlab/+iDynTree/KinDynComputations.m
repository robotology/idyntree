classdef KinDynComputations < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = KinDynComputations(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1382, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1383, self);
        self.swigPtr=[];
      end
    end
    function varargout = loadRobotModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1384, self, varargin{:});
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1385, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1386, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1387, self, varargin{:});
    end
    function varargout = setFrameVelocityRepresentation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1388, self, varargin{:});
    end
    function varargout = getFrameVelocityRepresentation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1389, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1390, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1391, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1392, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1393, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1394, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1395, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1396, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1397, self, varargin{:});
    end
    function varargout = getRobotModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1398, self, varargin{:});
    end
    function varargout = setJointPos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1399, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1400, self, varargin{:});
    end
    function varargout = getWorldBaseTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1401, self, varargin{:});
    end
    function varargout = getBaseTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1402, self, varargin{:});
    end
    function varargout = getJointPos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1403, self, varargin{:});
    end
    function varargout = getJointVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1404, self, varargin{:});
    end
    function varargout = getModelVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1405, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1406, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1407, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1408, self, varargin{:});
    end
    function varargout = getRelativeTransformExplicit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1409, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1410, self, varargin{:});
    end
    function varargout = getFrameVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1411, self, varargin{:});
    end
    function varargout = getFrameFreeFloatingJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1412, self, varargin{:});
    end
    function varargout = getCenterOfMassPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1413, self, varargin{:});
    end
    function varargout = getAverageVelocity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1414, self, varargin{:});
    end
    function varargout = getAverageVelocityJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1415, self, varargin{:});
    end
    function varargout = getLinearAngularMomentum(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1416, self, varargin{:});
    end
    function varargout = getLinearAngularMomentumJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1417, self, varargin{:});
    end
  end
  methods(Static)
  end
end
