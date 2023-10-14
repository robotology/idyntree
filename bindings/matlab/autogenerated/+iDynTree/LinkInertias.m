classdef LinkInertias < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = LinkInertias(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(809, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(810, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(811, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(812, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(813, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
