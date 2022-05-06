classdef ModelCalibrationHelper < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ModelCalibrationHelper(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1464, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1465, self);
        self.SwigClear();
      end
    end
    function varargout = loadModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1466, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1467, self, varargin{:});
    end
    function varargout = updateModelInertialParametersToString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1468, self, varargin{:});
    end
    function varargout = updateModelInertialParametersToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1469, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1470, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1471, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1472, self, varargin{:});
    end
  end
  methods(Static)
  end
end
