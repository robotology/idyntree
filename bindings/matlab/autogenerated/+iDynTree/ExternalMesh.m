classdef ExternalMesh < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1024, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1025, self, varargin{:});
    end
    function varargout = getFilename(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1026, self, varargin{:});
    end
    function varargout = setFilename(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1027, self, varargin{:});
    end
    function varargout = getScale(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1028, self, varargin{:});
    end
    function varargout = setScale(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1029, self, varargin{:});
    end
    function self = ExternalMesh(varargin)
      self@iDynTree.SolidShape(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1030, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
