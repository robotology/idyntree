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
        tmp = iDynTreeMEX(1469, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1470, self);
        self.SwigClear();
      end
    end
    function varargout = exportingOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1471, self, varargin{:});
    end
    function varargout = setExportingOptions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1472, self, varargin{:});
    end
    function varargout = init(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1473, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1474, self, varargin{:});
    end
    function varargout = sensors(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1475, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1476, self, varargin{:});
    end
    function varargout = exportModelToString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1477, self, varargin{:});
    end
    function varargout = exportModelToFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1478, self, varargin{:});
    end
  end
  methods(Static)
  end
end
