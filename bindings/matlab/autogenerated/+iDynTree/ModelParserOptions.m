classdef ModelParserOptions < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = addSensorFramesAsAdditionalFrames(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1424, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1425, self, varargin{1});
      end
    end
    function varargout = originalFilename(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1426, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1427, self, varargin{1});
      end
    end
    function self = ModelParserOptions(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1428, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1429, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
