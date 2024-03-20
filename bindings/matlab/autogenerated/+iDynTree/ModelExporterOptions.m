classdef ModelExporterOptions < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = baseLink(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1542, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1543, self, varargin{1});
      end
    end
    function varargout = exportFirstBaseLinkAdditionalFrameAsFakeURDFBase(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1544, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1545, self, varargin{1});
      end
    end
    function varargout = robotExportedName(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1546, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1547, self, varargin{1});
      end
    end
    function varargout = xmlBlobs(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1548, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1549, self, varargin{1});
      end
    end
    function self = ModelExporterOptions(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1550, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1551, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
