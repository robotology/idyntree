classdef SolidShape < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1089, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1090, self, varargin{:});
    end
    function varargout = name(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1091, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1092, self, varargin{1});
      end
    end
    function varargout = link_H_geometry(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1093, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1094, self, varargin{1});
      end
    end
    function varargout = material(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1095, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1096, self, varargin{1});
      end
    end
    function varargout = isSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1097, self, varargin{:});
    end
    function varargout = isBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1098, self, varargin{:});
    end
    function varargout = isCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1099, self, varargin{:});
    end
    function varargout = isExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1100, self, varargin{:});
    end
    function varargout = asSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1101, self, varargin{:});
    end
    function varargout = asBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1102, self, varargin{:});
    end
    function varargout = asCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1103, self, varargin{:});
    end
    function varargout = asExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1104, self, varargin{:});
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
