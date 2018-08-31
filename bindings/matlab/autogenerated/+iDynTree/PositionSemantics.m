classdef PositionSemantics < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = PositionSemantics(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(354, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setToUnknown(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(355, self, varargin{:});
    end
    function varargout = getPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(356, self, varargin{:});
    end
    function varargout = getBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(357, self, varargin{:});
    end
    function varargout = getReferencePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(358, self, varargin{:});
    end
    function varargout = getRefBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(359, self, varargin{:});
    end
    function varargout = getCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(360, self, varargin{:});
    end
    function varargout = setPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(361, self, varargin{:});
    end
    function varargout = setBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(362, self, varargin{:});
    end
    function varargout = setReferencePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(363, self, varargin{:});
    end
    function varargout = setRefBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(364, self, varargin{:});
    end
    function varargout = setCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(365, self, varargin{:});
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(366, self, varargin{:});
    end
    function varargout = changeRefPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(367, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(370, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(371, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(372, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(368, varargin{:});
    end
    function varargout = inverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(369, varargin{:});
    end
  end
end
