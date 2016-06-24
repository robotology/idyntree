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
        tmp = iDynTreeMEX(974, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(975, self);
        self.swigPtr=[];
      end
    end
    function varargout = addSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(976, self, varargin{:});
    end
    function varargout = setSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(977, self, varargin{:});
    end
    function varargout = getSerialization(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(978, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(979, self, varargin{:});
    end
    function varargout = getSensorIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(980, self, varargin{:});
    end
    function varargout = getSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(981, self, varargin{:});
    end
    function varargout = getSixAxisForceTorqueSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(982, self, varargin{:});
    end
    function varargout = getAccelerometerSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(983, self, varargin{:});
    end
    function varargout = getGyroscopeSensor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(984, self, varargin{:});
    end
  end
  methods(Static)
  end
end
