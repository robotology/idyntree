classdef SensorsMeasurements < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SensorsMeasurements(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1242, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1243, self);
        self.swigPtr=[];
      end
    end
    function varargout = setNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1244, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1245, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1246, self, varargin{:});
    end
    function varargout = toVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1247, self, varargin{:});
    end
    function varargout = setMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1248, self, varargin{:});
    end
    function varargout = getMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1249, self, varargin{:});
    end
    function varargout = getSizeOfAllSensorsMeasurements(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1250, self, varargin{:});
    end
  end
  methods(Static)
  end
end
