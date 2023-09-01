classdef DHLink < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = A(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1292, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1293, self, varargin{1});
      end
    end
    function varargout = D(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1294, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1295, self, varargin{1});
      end
    end
    function varargout = Alpha(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1296, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1297, self, varargin{1});
      end
    end
    function varargout = Offset(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1298, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1299, self, varargin{1});
      end
    end
    function varargout = Min(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1300, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1301, self, varargin{1});
      end
    end
    function varargout = Max(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1302, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1303, self, varargin{1});
      end
    end
    function self = DHLink(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1304, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1305, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
