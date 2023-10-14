classdef ExternalMesh < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1100, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1101, self, varargin{:});
    end
    function varargout = getFilename(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1102, self, varargin{:});
    end
    function varargout = getPackageDirs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1103, self, varargin{:});
    end
    function varargout = getFileLocationOnLocalFileSystem(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1104, self, varargin{:});
    end
    function varargout = setFilename(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1105, self, varargin{:});
    end
    function varargout = setPackageDirs(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1106, self, varargin{:});
    end
    function varargout = getScale(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1107, self, varargin{:});
    end
    function varargout = setScale(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1108, self, varargin{:});
    end
    function self = ExternalMesh(varargin)
      self@iDynTree.SolidShape(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1109, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
