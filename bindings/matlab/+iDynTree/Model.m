classdef Model < SwigRef
  methods
    function self = Model(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(669, varargin{:});
        tmp = iDynTreeMATLAB_wrap(669, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(670, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(671, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(672, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(673, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(674, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(675, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(676, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(677, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(678, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(679, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(680, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(681, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(682, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(683, self, varargin{:});
    end
  end
  methods(Static)
  end
end
