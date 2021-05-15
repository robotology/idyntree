classdef VisualizerOptions < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = verbose(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1894, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1895, self, varargin{1});
      end
    end
    function varargout = winWidth(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1896, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1897, self, varargin{1});
      end
    end
    function varargout = winHeight(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1898, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1899, self, varargin{1});
      end
    end
    function varargout = rootFrameArrowsDimension(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1900, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1901, self, varargin{1});
      end
    end
    function self = VisualizerOptions(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1902, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1903, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
