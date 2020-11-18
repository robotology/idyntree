classdef PositionRaw < iDynTree.Vector3
  methods
    function self = PositionRaw(varargin)
      self@iDynTree.Vector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(404, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(405, self, varargin{:});
    end
    function varargout = changeRefPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(406, self, varargin{:});
    end
    function varargout = changePointOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(409, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(410, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(411, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(412, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(407, varargin{:});
    end
    function varargout = inverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(408, varargin{:});
    end
  end
end
