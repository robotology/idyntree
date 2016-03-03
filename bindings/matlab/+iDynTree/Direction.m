classdef Direction < iDynTree.Vector3
  methods
    function self = Direction(varargin)
      self@iDynTree.Vector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(484, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = Normalize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(485, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(486, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(487, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(489, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = Default(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(488, varargin{:});
    end
  end
end
