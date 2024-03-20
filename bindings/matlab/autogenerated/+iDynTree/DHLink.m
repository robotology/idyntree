classdef DHLink < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1491, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1492, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1493, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1494, self, varargin{1});
      end
    end
    function varargout = Alpha(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1495, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1496, self, varargin{1});
      end
    end
    function varargout = Offset(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1497, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1498, self, varargin{1});
      end
    end
    function varargout = Min(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1499, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1500, self, varargin{1});
      end
    end
    function varargout = Max(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1501, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1502, self, varargin{1});
      end
    end
    function self = DHLink(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1503, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1504, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
