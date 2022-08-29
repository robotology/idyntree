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
        tmp = iDynTreeMEX(1169, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1170, self);
        self.SwigClear();
      end
    end
    function varargout = splitModelAlongJoints(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1171, self, varargin{:});
    end
    function varargout = setNrOfSubModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1172, self, varargin{:});
    end
    function varargout = getNrOfSubModels(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1173, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1174, self, varargin{:});
    end
    function varargout = getTraversal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1175, self, varargin{:});
    end
    function varargout = getSubModelOfLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1176, self, varargin{:});
    end
    function varargout = getSubModelOfFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1177, self, varargin{:});
    end
  end
  methods(Static)
  end
end
