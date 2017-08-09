classdef TransformSemantics < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = TransformSemantics(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(761, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getRotationSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(762, self, varargin{:});
    end
    function varargout = getPositionSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(763, self, varargin{:});
    end
    function varargout = setRotationSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(764, self, varargin{:});
    end
    function varargout = setPositionSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(765, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(766, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(767, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(768, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
