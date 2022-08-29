classdef DHLink < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1264, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1265, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1266, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1267, self, varargin{1});
      end
    end
    function varargout = Alpha(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1268, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1269, self, varargin{1});
      end
    end
    function varargout = Offset(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1270, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1271, self, varargin{1});
      end
    end
    function varargout = Min(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1272, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1273, self, varargin{1});
      end
    end
    function varargout = Max(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1274, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1275, self, varargin{1});
      end
    end
    function self = DHLink(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1276, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1277, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
