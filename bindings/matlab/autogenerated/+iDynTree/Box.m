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
    function varargout = getX(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1086, self, varargin{:});
    end
    function varargout = setX(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1087, self, varargin{:});
    end
    function varargout = getY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1088, self, varargin{:});
    end
    function varargout = setY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1089, self, varargin{:});
    end
    function varargout = getZ(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1090, self, varargin{:});
    end
    function varargout = setZ(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1091, self, varargin{:});
    end
    function self = Box(varargin)
      self@iDynTree.SolidShape(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
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
