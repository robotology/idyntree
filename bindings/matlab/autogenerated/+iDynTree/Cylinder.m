classdef Cylinder < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1020, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1021, self, varargin{:});
    end
    function varargout = getLength(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1022, self, varargin{:});
    end
    function varargout = setLength(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1023, self, varargin{:});
    end
    function varargout = getRadius(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1024, self, varargin{:});
    end
    function varargout = setRadius(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1025, self, varargin{:});
    end
    function self = Cylinder(varargin)
      self@iDynTree.SolidShape(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1026, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
