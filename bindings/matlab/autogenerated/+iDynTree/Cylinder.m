classdef Cylinder < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1119, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1120, self, varargin{:});
    end
    function varargout = length(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1121, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1122, self, varargin{1});
      end
    end
    function varargout = radius(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1123, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1124, self, varargin{1});
      end
    end
    function self = Cylinder(varargin)
      self@iDynTree.SolidShape(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1125, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
