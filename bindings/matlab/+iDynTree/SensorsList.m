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
        tmp = iDynTreeMEX(1194, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1195, self);
        self.swigPtr=[];
      end
    end
    function varargout = addSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1196, self, varargin{:});
    end
    function varargout = setSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1197, self, varargin{:});
    end
    function varargout = getSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1198, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1199, self, varargin{:});
    end
    function varargout = getSensorIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1200, self, varargin{:});
    end
    function varargout = getSizeOfAllSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1201, self, varargin{:});
    end
    function varargout = getSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1202, self, varargin{:});
    end
    function varargout = removeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1203, self, varargin{:});
    end
    function varargout = removeAllSensorsOfType(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1204, self, varargin{:});
    end
    function varargout = getSixAxisForceTorqueSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1205, self, varargin{:});
    end
    function varargout = getAccelerometerSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1206, self, varargin{:});
    end
    function varargout = getGyroscopeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1207, self, varargin{:});
    end
  end
  methods(Static)
  end
end
