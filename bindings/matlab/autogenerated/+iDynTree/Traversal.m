classdef Traversal < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Traversal(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1131, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1132, self);
        self.SwigClear();
      end
    end
    function varargout = getNrOfVisitedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1133, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1134, self, varargin{:});
    end
    function varargout = getBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1135, self, varargin{:});
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1136, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1137, self, varargin{:});
    end
    function varargout = getParentLinkFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1138, self, varargin{:});
    end
    function varargout = getParentJointFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1139, self, varargin{:});
    end
    function varargout = getTraversalIndexFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1140, self, varargin{:});
    end
    function varargout = reset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1141, self, varargin{:});
    end
    function varargout = addTraversalBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1142, self, varargin{:});
    end
    function varargout = addTraversalElement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1143, self, varargin{:});
    end
    function varargout = isParentOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1144, self, varargin{:});
    end
    function varargout = getChildLinkIndexFromJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1145, self, varargin{:});
    end
    function varargout = getParentLinkIndexFromJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1146, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1147, self, varargin{:});
    end
  end
  methods(Static)
  end
end
