classdef SixAxisForceTorqueSensor < iDynTree.JointSensor
  methods
    function self = SixAxisForceTorqueSensor(varargin)
      self@iDynTree.JointSensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1452, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1453, self);
        self.SwigClear();
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1454, self, varargin{:});
    end
    function varargout = setFirstLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1455, self, varargin{:});
    end
    function varargout = setSecondLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1456, self, varargin{:});
    end
    function varargout = getFirstLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1457, self, varargin{:});
    end
    function varargout = getSecondLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1458, self, varargin{:});
    end
    function varargout = setFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1459, self, varargin{:});
    end
    function varargout = setSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1460, self, varargin{:});
    end
    function varargout = getFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1461, self, varargin{:});
    end
    function varargout = getSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1462, self, varargin{:});
    end
    function varargout = setParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1463, self, varargin{:});
    end
    function varargout = setParentJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1464, self, varargin{:});
    end
    function varargout = setAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1465, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1466, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1467, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1468, self, varargin{:});
    end
    function varargout = getParentJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1469, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1470, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1471, self, varargin{:});
    end
    function varargout = updateIndices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1472, self, varargin{:});
    end
    function varargout = updateIndeces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1473, self, varargin{:});
    end
    function varargout = getAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1474, self, varargin{:});
    end
    function varargout = isLinkAttachedToSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1475, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1476, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1477, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLinkMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1478, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLinkInverseMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1479, self, varargin{:});
    end
    function varargout = predictMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1480, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1481, self, varargin{:});
    end
  end
  methods(Static)
  end
end
