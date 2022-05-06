classdef DHLink < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1249, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1250, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1251, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1252, self, varargin{1});
      end
    end
    function varargout = Alpha(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1253, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1254, self, varargin{1});
      end
    end
    function varargout = Offset(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1255, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1256, self, varargin{1});
      end
    end
    function varargout = Min(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1257, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1258, self, varargin{1});
      end
    end
    function varargout = Max(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1259, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1260, self, varargin{1});
      end
    end
    function self = DHLink(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1261, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1262, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
