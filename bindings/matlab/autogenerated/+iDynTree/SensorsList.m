classdef SensorsList < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SensorsList(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1244, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1245, self);
        self.SwigClear();
      end
    end
    function varargout = addSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1246, self, varargin{:});
    end
    function varargout = setSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1247, self, varargin{:});
    end
    function varargout = getSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1248, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1249, self, varargin{:});
    end
    function varargout = getSensorIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1250, self, varargin{:});
    end
    function varargout = getSizeOfAllSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1251, self, varargin{:});
    end
    function varargout = getSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1252, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1253, self, varargin{:});
    end
    function varargout = removeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1254, self, varargin{:});
    end
    function varargout = removeAllSensorsOfType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1255, self, varargin{:});
    end
    function varargout = getSixAxisForceTorqueSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1256, self, varargin{:});
    end
    function varargout = getAccelerometerSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1257, self, varargin{:});
    end
    function varargout = getGyroscopeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1258, self, varargin{:});
    end
    function varargout = getThreeAxisAngularAccelerometerSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1259, self, varargin{:});
    end
    function varargout = getThreeAxisForceTorqueContactSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1260, self, varargin{:});
    end
  end
  methods(Static)
  end
end
