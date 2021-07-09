classdef ExternalMesh < iDynTree.SolidShape
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1027, self);
        self.SwigClear();
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1028, self, varargin{:});
    end
    function varargout = getFilename(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1029, self, varargin{:});
    end
    function varargout = getFileLocationOnLocalFileSystem(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1030, self, varargin{:});
    end
    function varargout = setFilename(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1031, self, varargin{:});
    end
    function varargout = getScale(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1032, self, varargin{:});
    end
    function varargout = setScale(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1033, self, varargin{:});
    end
    function self = ExternalMesh(varargin)
      self@iDynTree.SolidShape(iDynTreeSwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1034, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
