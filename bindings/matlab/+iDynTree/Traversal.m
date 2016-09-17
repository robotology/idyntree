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
        tmp = iDynTreeMEX(894, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(895, self);
        self.swigPtr=[];
      end
    end
    function varargout = getNrOfVisitedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(896, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(897, self, varargin{:});
    end
    function varargout = getBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(898, self, varargin{:});
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(899, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(900, self, varargin{:});
    end
    function varargout = getParentLinkFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(901, self, varargin{:});
    end
    function varargout = getParentJointFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(902, self, varargin{:});
    end
    function varargout = getTraversalIndexFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(903, self, varargin{:});
    end
    function varargout = reset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(904, self, varargin{:});
    end
    function varargout = addTraversalBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(905, self, varargin{:});
    end
    function varargout = addTraversalElement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(906, self, varargin{:});
    end
    function varargout = isParentOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(907, self, varargin{:});
    end
    function varargout = getChildLinkIndexFromJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(908, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(909, self, varargin{:});
    end
  end
  methods(Static)
  end
end
