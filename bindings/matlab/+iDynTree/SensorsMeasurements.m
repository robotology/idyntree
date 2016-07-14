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
        tmp = iDynTreeMEX(995, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(996, self);
        self.swigPtr=[];
      end
    end
    function varargout = setNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(997, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(998, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(999, self, varargin{:});
    end
    function varargout = toVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1000, self, varargin{:});
    end
    function varargout = setMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1001, self, varargin{:});
    end
    function varargout = getMeasurement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1002, self, varargin{:});
    end
  end
  methods(Static)
  end
end
