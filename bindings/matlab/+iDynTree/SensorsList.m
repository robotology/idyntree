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
        tmp = iDynTreeMEX(1104, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1105, self);
        self.swigPtr=[];
      end
    end
    function varargout = addSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1106, self, varargin{:});
    end
    function varargout = setSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1107, self, varargin{:});
    end
    function varargout = getSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1108, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1109, self, varargin{:});
    end
    function varargout = getSensorIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1110, self, varargin{:});
    end
    function varargout = getSizeOfAllSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1111, self, varargin{:});
    end
    function varargout = getSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1112, self, varargin{:});
    end
    function varargout = removeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1113, self, varargin{:});
    end
    function varargout = removeAllSensorsOfType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1114, self, varargin{:});
    end
    function varargout = getSixAxisForceTorqueSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1115, self, varargin{:});
    end
    function varargout = getAccelerometerSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1116, self, varargin{:});
    end
    function varargout = getGyroscopeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1117, self, varargin{:});
    end
  end
  methods(Static)
  end
end
