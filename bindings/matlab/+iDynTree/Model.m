classdef Model < SwigRef
  methods
    function self = Model(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(645, varargin{:});
        tmp = iDynTreeMATLAB_wrap(645, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(646, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(647, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(648, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(649, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(650, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(651, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(652, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(653, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(654, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(655, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(656, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(657, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(658, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(659, self, varargin{:});
    end
  end
  methods(Static)
  end
end
