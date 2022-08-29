classdef SensorsList < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SensorsList(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1321, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1322, self);
        self.SwigClear();
      end
    end
    function varargout = addSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1323, self, varargin{:});
    end
    function varargout = setSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1324, self, varargin{:});
    end
    function varargout = getSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1325, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1326, self, varargin{:});
    end
    function varargout = getSensorIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1327, self, varargin{:});
    end
    function varargout = getSizeOfAllSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1328, self, varargin{:});
    end
    function varargout = getSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1329, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1330, self, varargin{:});
    end
    function varargout = removeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1331, self, varargin{:});
    end
    function varargout = removeAllSensorsOfType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1332, self, varargin{:});
    end
    function varargout = getSixAxisForceTorqueSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1333, self, varargin{:});
    end
    function varargout = getAccelerometerSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1334, self, varargin{:});
    end
    function varargout = getGyroscopeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1335, self, varargin{:});
    end
    function varargout = getThreeAxisAngularAccelerometerSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1336, self, varargin{:});
    end
    function varargout = getThreeAxisForceTorqueContactSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1337, self, varargin{:});
    end
  end
  methods(Static)
  end
end
