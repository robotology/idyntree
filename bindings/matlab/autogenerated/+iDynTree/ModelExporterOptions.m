classdef ModelExporterOptions < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = baseLink(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1397, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1398, self, varargin{1});
      end
    end
    function varargout = exportFirstBaseLinkAdditionalFrameAsFakeURDFBase(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1399, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1400, self, varargin{1});
      end
    end
    function self = ModelExporterOptions(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1401, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1402, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
