classdef Model < SwigRef
  methods
    function self = Model(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(684, varargin{:});
        tmp = iDynTreeMATLAB_wrap(684, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(685, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(686, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(687, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(688, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(689, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(690, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(691, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(692, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(693, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(694, self, varargin{:});
    end
    function varargout = getNrOfDOFs(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(695, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(696, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(697, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(698, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(699, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(700, self, varargin{:});
    end
  end
  methods(Static)
  end
end
