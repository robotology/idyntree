classdef SubModelDecomposition < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SubModelDecomposition(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1300, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1301, self);
        self.SwigClear();
      end
    end
    function varargout = splitModelAlongJoints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1302, self, varargin{:});
    end
    function varargout = setNrOfSubModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1303, self, varargin{:});
    end
    function varargout = getNrOfSubModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1304, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1305, self, varargin{:});
    end
    function varargout = getTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1306, self, varargin{:});
    end
    function varargout = getSubModelOfLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1307, self, varargin{:});
    end
    function varargout = getSubModelOfFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1308, self, varargin{:});
    end
  end
  methods(Static)
  end
end
