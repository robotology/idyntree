classdef SixAxisForceTorqueSensor < iDynTree.JointSensor
  methods
    function self = SixAxisForceTorqueSensor(varargin)
      self@iDynTree.JointSensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1270, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1271, self);
        self.SwigClear();
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1272, self, varargin{:});
    end
    function varargout = setFirstLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1273, self, varargin{:});
    end
    function varargout = setSecondLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1274, self, varargin{:});
    end
    function varargout = getFirstLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1275, self, varargin{:});
    end
    function varargout = getSecondLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1276, self, varargin{:});
    end
    function varargout = setFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1277, self, varargin{:});
    end
    function varargout = setSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1278, self, varargin{:});
    end
    function varargout = getFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1279, self, varargin{:});
    end
    function varargout = getSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1280, self, varargin{:});
    end
    function varargout = setParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1281, self, varargin{:});
    end
    function varargout = setParentJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1282, self, varargin{:});
    end
    function varargout = setAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1283, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1284, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1285, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1286, self, varargin{:});
    end
    function varargout = getParentJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1287, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1288, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1289, self, varargin{:});
    end
    function varargout = updateIndices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1290, self, varargin{:});
    end
    function varargout = updateIndeces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1291, self, varargin{:});
    end
    function varargout = getAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1292, self, varargin{:});
    end
    function varargout = isLinkAttachedToSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1293, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1294, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1295, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLinkMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1296, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLinkInverseMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1297, self, varargin{:});
    end
    function varargout = predictMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1298, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1299, self, varargin{:});
    end
  end
  methods(Static)
  end
end
