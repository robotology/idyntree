classdef LinkUnknownWrenchContacts < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = LinkUnknownWrenchContacts(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1465, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1466, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1467, self, varargin{:});
    end
    function varargout = getNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1468, self, varargin{:});
    end
    function varargout = setNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1469, self, varargin{:});
    end
    function varargout = addNewContactForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1470, self, varargin{:});
    end
    function varargout = addNewContactInFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1471, self, varargin{:});
    end
    function varargout = addNewUnknownFullWrenchInFrameOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1472, self, varargin{:});
    end
    function varargout = contactWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1473, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1474, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1475, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
