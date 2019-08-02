classdef LinkContactWrenches < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = LinkContactWrenches(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1243, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1244, self, varargin{:});
    end
    function varargout = getNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1245, self, varargin{:});
    end
    function varargout = setNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1246, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1247, self, varargin{:});
    end
    function varargout = contactWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1248, self, varargin{:});
    end
    function varargout = computeNetWrenches(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1249, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1250, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1251, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
