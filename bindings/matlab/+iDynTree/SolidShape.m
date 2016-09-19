classdef SolidShape < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(910, self);
        self.swigPtr=[];
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(911, self, varargin{:});
    end
    function varargout = name(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(912, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(913, self, varargin{1});
      end
    end
    function varargout = link_H_geometry(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(914, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(915, self, varargin{1});
      end
    end
    function varargout = material(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(916, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(917, self, varargin{1});
      end
    end
    function varargout = isSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(918, self, varargin{:});
    end
    function varargout = isBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(919, self, varargin{:});
    end
    function varargout = isCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(920, self, varargin{:});
    end
    function varargout = isExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(921, self, varargin{:});
    end
    function varargout = asSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(922, self, varargin{:});
    end
    function varargout = asBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(923, self, varargin{:});
    end
    function varargout = asCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(924, self, varargin{:});
    end
    function varargout = asExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(925, self, varargin{:});
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
