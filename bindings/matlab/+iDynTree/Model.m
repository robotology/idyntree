classdef Model < SwigRef
  methods
    function self = Model(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(438, varargin{:});
        tmp = iDynTreeMATLAB_wrap(438, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(439, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(440, self, varargin{:});
    end
    function varargout = getLinkName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(441, self, varargin{:});
    end
    function varargout = getLinkIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(442, self, varargin{:});
    end
    function varargout = addLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(443, self, varargin{:});
    end
    function varargout = getNrOfJoints(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(444, self, varargin{:});
    end
    function varargout = getJointName(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(445, self, varargin{:});
    end
    function varargout = getJointIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(446, self, varargin{:});
    end
    function varargout = addJoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(447, self, varargin{:});
    end
    function varargout = getNrOfNeighbors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(448, self, varargin{:});
    end
    function varargout = getNeighbor(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(449, self, varargin{:});
    end
    function varargout = setDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(450, self, varargin{:});
    end
    function varargout = getDefaultBaseLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(451, self, varargin{:});
    end
    function varargout = computeFullTreeTraversal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(452, self, varargin{:});
    end
  end
  methods(Static)
  end
end
