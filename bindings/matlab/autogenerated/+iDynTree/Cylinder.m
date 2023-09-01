classdef Cylinder < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1044, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1045, self, varargin{:});
    end
    function varargout = getLength(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1046, self, varargin{:});
    end
    function varargout = setLength(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1047, self, varargin{:});
    end
    function varargout = getRadius(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1048, self, varargin{:});
    end
    function varargout = setRadius(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1049, self, varargin{:});
    end
    function self = Cylinder(varargin)
      self@iDynTree.SolidShape(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1050, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
