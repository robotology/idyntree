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
        tmp = iDynTreeMEX(1374, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1375, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1376, self, varargin{:});
    end
    function varargout = getNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1377, self, varargin{:});
    end
    function varargout = setNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1378, self, varargin{:});
    end
    function varargout = addNewContactForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1379, self, varargin{:});
    end
    function varargout = addNewContactInFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1380, self, varargin{:});
    end
    function varargout = addNewUnknownFullWrenchInFrameOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1381, self, varargin{:});
    end
    function varargout = contactWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1382, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1383, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1384, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
