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
        tmp = iDynTreeMEX(756, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getRotationSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(757, self, varargin{:});
    end
    function varargout = getPositionSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(758, self, varargin{:});
    end
    function varargout = setRotationSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(759, self, varargin{:});
    end
    function varargout = setPositionSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(760, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(761, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(762, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(763, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
