classdef SixAxisForceTorqueSensor < iDynTree.JointSensor
  methods
    function self = SixAxisForceTorqueSensor(varargin)
      self@iDynTree.JointSensor(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1172, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1173, self);
        self.swigPtr=[];
      end
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1174, self, varargin{:});
    end
    function varargout = setFirstLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1175, self, varargin{:});
    end
    function varargout = setSecondLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1176, self, varargin{:});
    end
    function varargout = getFirstLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1177, self, varargin{:});
    end
    function varargout = getSecondLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1178, self, varargin{:});
    end
    function varargout = setFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1179, self, varargin{:});
    end
    function varargout = setSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1180, self, varargin{:});
    end
    function varargout = getFirstLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1181, self, varargin{:});
    end
    function varargout = getSecondLinkName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1182, self, varargin{:});
    end
    function varargout = setParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1183, self, varargin{:});
    end
    function varargout = setParentJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1184, self, varargin{:});
    end
    function varargout = setAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1185, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1186, self, varargin{:});
    end
    function varargout = getSensorType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1187, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1188, self, varargin{:});
    end
    function varargout = getParentJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1189, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1190, self, varargin{:});
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1191, self, varargin{:});
    end
    function varargout = updateIndeces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1192, self, varargin{:});
    end
    function varargout = getAppliedWrenchLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1193, self, varargin{:});
    end
    function varargout = isLinkAttachedToSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1194, self, varargin{:});
    end
    function varargout = getLinkSensorTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1195, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1196, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLinkMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1197, self, varargin{:});
    end
    function varargout = getWrenchAppliedOnLinkInverseMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1198, self, varargin{:});
    end
    function varargout = predictMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1199, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1200, self, varargin{:});
    end
  end
  methods(Static)
  end
end
