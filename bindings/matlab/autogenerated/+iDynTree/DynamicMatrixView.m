classdef DynamicMatrixView < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = DynamicMatrixView(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(769, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = storageOrder(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(770, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(771, self, varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(772, self, varargin{:});
    end
    function varargout = cols(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(773, self, varargin{:});
    end
    function varargout = block(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(774, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(775, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
