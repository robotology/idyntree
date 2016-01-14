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
        tmp = iDynTreeMEX(720, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(721, self);
        self.swigPtr=[];
      end
    end
    function varargout = getNrOfVisitedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(722, self, varargin{:});
    end
    function varargout = getLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(723, self, varargin{:});
    end
    function varargout = getParentLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(724, self, varargin{:});
    end
    function varargout = getParentJoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(725, self, varargin{:});
    end
    function varargout = getParentLinkFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(726, self, varargin{:});
    end
    function varargout = getParentJointFromLinkIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(727, self, varargin{:});
    end
    function varargout = reset(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(728, self, varargin{:});
    end
    function varargout = addTraversalBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(729, self, varargin{:});
    end
    function varargout = addTraversalElement(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(730, self, varargin{:});
    end
  end
  methods(Static)
  end
end
