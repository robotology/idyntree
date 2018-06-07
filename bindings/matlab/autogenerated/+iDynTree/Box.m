classdef Box < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1084, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1085, self, varargin{:});
    end
    function varargout = x(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1086, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1087, self, varargin{1});
      end
    end
    function varargout = y(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1088, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1089, self, varargin{1});
      end
    end
    function varargout = z(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1090, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1091, self, varargin{1});
      end
    end
    function self = Box(varargin)
      self@iDynTree.SolidShape(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1092, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
