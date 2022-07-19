classdef VisualizerOptions < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = verbose(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1956, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1957, self, varargin{1});
      end
    end
    function varargout = winWidth(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1958, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1959, self, varargin{1});
      end
    end
    function varargout = winHeight(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1960, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1961, self, varargin{1});
      end
    end
    function varargout = rootFrameArrowsDimension(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1962, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1963, self, varargin{1});
      end
    end
    function self = VisualizerOptions(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1964, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1965, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
