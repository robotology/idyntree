classdef SolidShape < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1022, self);
        self.swigPtr=[];
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1023, self, varargin{:});
    end
    function varargout = name(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1024, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1025, self, varargin{1});
      end
    end
    function varargout = link_H_geometry(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1026, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1027, self, varargin{1});
      end
    end
    function varargout = material(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1028, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1029, self, varargin{1});
      end
    end
    function varargout = isSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1030, self, varargin{:});
    end
    function varargout = isBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1031, self, varargin{:});
    end
    function varargout = isCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1032, self, varargin{:});
    end
    function varargout = isExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1033, self, varargin{:});
    end
    function varargout = asSphere(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1034, self, varargin{:});
    end
    function varargout = asBox(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1035, self, varargin{:});
    end
    function varargout = asCylinder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1036, self, varargin{:});
    end
    function varargout = asExternalMesh(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1037, self, varargin{:});
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
