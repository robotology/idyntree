classdef Traversal < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Traversal(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(828, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(829, self);
        self.swigPtr=[];
      end
    end
    function varargout = getNrOfVisitedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(830, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(831, self, varargin{:});
    end
    function varargout = getBaseLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(832, self, varargin{:});
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(833, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(834, self, varargin{:});
    end
    function varargout = getParentLinkFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(835, self, varargin{:});
    end
    function varargout = getParentJointFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(836, self, varargin{:});
    end
    function varargout = getTraversalIndexFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(837, self, varargin{:});
    end
    function varargout = reset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(838, self, varargin{:});
    end
    function varargout = addTraversalBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(839, self, varargin{:});
    end
    function varargout = addTraversalElement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(840, self, varargin{:});
    end
    function varargout = isParentOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(841, self, varargin{:});
    end
    function varargout = getChildLinkIndexFromJointIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(842, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(843, self, varargin{:});
    end
  end
  methods(Static)
  end
end
