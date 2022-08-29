classdef VisualizerOptions < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = verbose(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1969, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1970, self, varargin{1});
      end
    end
    function varargout = winWidth(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1971, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1972, self, varargin{1});
      end
    end
    function varargout = winHeight(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1973, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1974, self, varargin{1});
      end
    end
    function varargout = rootFrameArrowsDimension(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1975, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1976, self, varargin{1});
      end
    end
    function self = VisualizerOptions(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1977, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1978, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
