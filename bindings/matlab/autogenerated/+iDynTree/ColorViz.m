classdef ColorViz < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = r(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1586, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1587, self, varargin{1});
      end
    end
    function varargout = g(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1588, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1589, self, varargin{1});
      end
    end
    function varargout = b(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1590, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1591, self, varargin{1});
      end
    end
    function varargout = a(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1592, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1593, self, varargin{1});
      end
    end
    function self = ColorViz(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1594, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1595, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
