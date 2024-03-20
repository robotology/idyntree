classdef Sphere < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1079, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1080, self, varargin{:});
    end
    function varargout = getRadius(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1081, self, varargin{:});
    end
    function varargout = setRadius(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1082, self, varargin{:});
    end
    function self = Sphere(varargin)
      self@iDynTree.SolidShape(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1083, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
