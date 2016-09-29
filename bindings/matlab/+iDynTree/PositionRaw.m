classdef PositionRaw < iDynTree.Vector3
  methods
    function self = PositionRaw(varargin)
      self@iDynTree.Vector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(279, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(280, self, varargin{:});
    end
    function varargout = changeRefPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(281, self, varargin{:});
    end
    function varargout = changePointOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(284, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(285, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(286, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(287, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(282, varargin{:});
    end
    function varargout = inverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(283, varargin{:});
    end
  end
end
