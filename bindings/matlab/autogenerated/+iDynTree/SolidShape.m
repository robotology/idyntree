classdef SolidShape < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(986, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(987, self, varargin{:});
    end
    function varargout = getName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(988, self, varargin{:});
    end
    function varargout = setName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(989, self, varargin{:});
    end
    function varargout = isNameValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(990, self, varargin{:});
    end
    function varargout = getLink_H_geometry(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(991, self, varargin{:});
    end
    function varargout = setLink_H_geometry(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(992, self, varargin{:});
    end
    function varargout = isMaterialSet(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(993, self, varargin{:});
    end
    function varargout = getMaterial(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(994, self, varargin{:});
    end
    function varargout = setMaterial(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(995, self, varargin{:});
    end
    function varargout = isSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(996, self, varargin{:});
    end
    function varargout = isBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(997, self, varargin{:});
    end
    function varargout = isCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(998, self, varargin{:});
    end
    function varargout = isExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(999, self, varargin{:});
    end
    function varargout = asSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1000, self, varargin{:});
    end
    function varargout = asBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1001, self, varargin{:});
    end
    function varargout = asCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1002, self, varargin{:});
    end
    function varargout = asExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1003, self, varargin{:});
    end
    function self = SolidShape(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
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
