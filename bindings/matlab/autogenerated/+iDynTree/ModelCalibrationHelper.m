classdef ModelCalibrationHelper < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ModelCalibrationHelper(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1595, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1596, self);
        self.SwigClear();
      end
    end
    function varargout = loadModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1597, self, varargin{:});
    end
    function varargout = loadModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1598, self, varargin{:});
    end
    function varargout = updateModelInertialParametersToString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1599, self, varargin{:});
    end
    function varargout = updateModelInertialParametersToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1600, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1601, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1602, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1603, self, varargin{:});
    end
  end
  methods(Static)
  end
end
