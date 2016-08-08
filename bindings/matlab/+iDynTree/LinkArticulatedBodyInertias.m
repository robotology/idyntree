classdef LinkArticulatedBodyInertias < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = LinkArticulatedBodyInertias(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(682, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(683, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(684, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(685, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(686, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
