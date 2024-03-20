classdef ModelSolidShapes < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1110, self);
        self.SwigClear();
      end
    end
    function self = ModelSolidShapes(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1111, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1112, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1113, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1114, self, varargin{:});
    end
    function varargout = clearSingleLinkSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1115, self, varargin{:});
    end
    function varargout = addSingleLinkSolidShape(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1116, self, varargin{:});
    end
    function varargout = getLinkSolidShapes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1117, self, varargin{:});
    end
  end
  methods(Static)
  end
end
