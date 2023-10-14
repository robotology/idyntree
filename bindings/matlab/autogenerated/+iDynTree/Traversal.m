classdef Traversal < iDynTreeSwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Traversal(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'iDynTreeSwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1035, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1036, self);
        self.SwigClear();
      end
    end
    function varargout = getNrOfVisitedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1037, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1038, self, varargin{:});
    end
    function varargout = getBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1039, self, varargin{:});
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1040, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1041, self, varargin{:});
    end
    function varargout = getParentLinkFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1042, self, varargin{:});
    end
    function varargout = getParentJointFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1043, self, varargin{:});
    end
    function varargout = getTraversalIndexFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1044, self, varargin{:});
    end
    function varargout = reset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1045, self, varargin{:});
    end
    function varargout = addTraversalBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1046, self, varargin{:});
    end
    function varargout = addTraversalElement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1047, self, varargin{:});
    end
    function varargout = isParentOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1048, self, varargin{:});
    end
    function varargout = getChildLinkIndexFromJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1049, self, varargin{:});
    end
    function varargout = getParentLinkIndexFromJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1050, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1051, self, varargin{:});
    end
  end
  methods(Static)
  end
end
