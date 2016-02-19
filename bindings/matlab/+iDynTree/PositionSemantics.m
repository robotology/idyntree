classdef PositionSemantics < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = PositionSemantics(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(176, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = setToUnknown(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(177, self, varargin{:});
    end
    function varargout = getPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(178, self, varargin{:});
    end
    function varargout = getBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(179, self, varargin{:});
    end
    function varargout = getReferencePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(180, self, varargin{:});
    end
    function varargout = getRefBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(181, self, varargin{:});
    end
    function varargout = getCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(182, self, varargin{:});
    end
    function varargout = setPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(183, self, varargin{:});
    end
    function varargout = setBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(184, self, varargin{:});
    end
    function varargout = setReferencePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(185, self, varargin{:});
    end
    function varargout = setRefBody(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(186, self, varargin{:});
    end
    function varargout = setCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(187, self, varargin{:});
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(188, self, varargin{:});
    end
    function varargout = changeRefPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(189, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(192, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(193, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(194, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(190, varargin{:});
    end
    function varargout = inverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(191, varargin{:});
    end
  end
end
