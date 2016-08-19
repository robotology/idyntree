classdef IndexRange < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = offset(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(71, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(72, self, varargin{1});
      end
    end
    function varargout = size(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(73, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(74, self, varargin{1});
      end
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(75, self, varargin{:});
    end
    function self = IndexRange(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(77, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(78, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = InvalidRange(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(76, varargin{:});
    end
  end
end
