classdef SixAxisForceTorqueSensor < iDynTree.JointSensor
  methods
    function self = SixAxisForceTorqueSensor(varargin)
      self@iDynTree.JointSensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1217, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1218, self);
        self.swigPtr=[];
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1219, self, varargin{:});
    end
    function varargout = setFirstLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1220, self, varargin{:});
    end
    function varargout = setSecondLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1221, self, varargin{:});
    end
    function varargout = getFirstLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1222, self, varargin{:});
    end
    function varargout = getSecondLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1223, self, varargin{:});
    end
    function varargout = setFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1224, self, varargin{:});
    end
    function varargout = setSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1225, self, varargin{:});
    end
    function varargout = getFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1226, self, varargin{:});
    end
    function varargout = getSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1227, self, varargin{:});
    end
    function varargout = setParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1228, self, varargin{:});
    end
    function varargout = setParentJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1229, self, varargin{:});
    end
    function varargout = setAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1230, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1231, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1232, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1233, self, varargin{:});
    end
    function varargout = getParentJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1234, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1235, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1236, self, varargin{:});
    end
    function varargout = updateIndeces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1237, self, varargin{:});
    end
    function varargout = getAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1238, self, varargin{:});
    end
    function varargout = isLinkAttachedToSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1239, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1240, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1241, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLinkMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1242, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLinkInverseMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1243, self, varargin{:});
    end
    function varargout = predictMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1244, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1245, self, varargin{:});
    end
  end
  methods(Static)
  end
end
