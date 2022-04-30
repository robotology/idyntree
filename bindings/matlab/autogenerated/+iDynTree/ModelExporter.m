classdef ModelExporter < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = ModelExporter(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1454, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1455, self);
        self.SwigClear();
      end
    end
    function varargout = exportingOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1456, self, varargin{:});
    end
    function varargout = setExportingOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1457, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1458, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1459, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1460, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1461, self, varargin{:});
    end
    function varargout = exportModelToString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1462, self, varargin{:});
    end
    function varargout = exportModelToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1463, self, varargin{:});
    end
  end
  methods(Static)
  end
end
