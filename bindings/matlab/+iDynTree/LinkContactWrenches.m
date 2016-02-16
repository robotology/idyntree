classdef LinkContactWrenches < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = LinkContactWrenches(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(824, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(825, self, varargin{:});
    end
    function varargout = getNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(826, self, varargin{:});
    end
    function varargout = setNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(827, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(828, self, varargin{:});
    end
    function varargout = contactWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(829, self, varargin{:});
    end
    function varargout = computeNetWrenches(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(830, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(831, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(832, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
