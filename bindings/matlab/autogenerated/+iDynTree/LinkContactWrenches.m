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
        tmp = iDynTreeMEX(1174, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1175, self, varargin{:});
    end
    function varargout = getNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1176, self, varargin{:});
    end
    function varargout = setNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1177, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1178, self, varargin{:});
    end
    function varargout = contactWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1179, self, varargin{:});
    end
    function varargout = computeNetWrenches(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1180, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1181, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1182, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
