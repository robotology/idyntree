classdef PixelViz < iDynTree.ColorViz
  methods
    function varargout = width(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1863, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1864, self, varargin{1});
      end
    end
    function varargout = height(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1865, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1866, self, varargin{1});
      end
    end
    function self = PixelViz(varargin)
      self@iDynTree.ColorViz(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1867, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1868, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
