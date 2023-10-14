classdef SolidShape < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1061, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1062, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1063, self, varargin{:});
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1064, self, varargin{:});
    end
    function varargout = isNameValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1065, self, varargin{:});
    end
    function varargout = getLink_H_geometry(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1066, self, varargin{:});
    end
    function varargout = setLink_H_geometry(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1067, self, varargin{:});
    end
    function varargout = isMaterialSet(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1068, self, varargin{:});
    end
    function varargout = getMaterial(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1069, self, varargin{:});
    end
    function varargout = setMaterial(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1070, self, varargin{:});
    end
    function varargout = isSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1071, self, varargin{:});
    end
    function varargout = isBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1072, self, varargin{:});
    end
    function varargout = isCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1073, self, varargin{:});
    end
    function varargout = isExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1074, self, varargin{:});
    end
    function varargout = asSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1075, self, varargin{:});
    end
    function varargout = asBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1076, self, varargin{:});
    end
    function varargout = asCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1077, self, varargin{:});
    end
    function varargout = asExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1078, self, varargin{:});
    end
    function self = SolidShape(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
