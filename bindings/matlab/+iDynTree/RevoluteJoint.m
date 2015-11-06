classdef RevoluteJoint < iDynTree.MovableJointImpl1
  methods
    function self = RevoluteJoint(varargin)
      self@iDynTree.MovableJointImpl1('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(657, varargin{:});
        tmp = iDynTreeMATLAB_wrap(657, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(658, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(659, self, varargin{:});
    end
    function varargout = setAttachedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(660, self, varargin{:});
    end
    function varargout = setRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(661, self, varargin{:});
    end
    function varargout = setAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(662, self, varargin{:});
    end
    function varargout = getFirstAttachedLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(663, self, varargin{:});
    end
    function varargout = getSecondAttachedLink(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(664, self, varargin{:});
    end
    function varargout = getAxis(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(665, self, varargin{:});
    end
    function varargout = getTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(666, self, varargin{:});
    end
    function varargout = computeLinkPosVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(667, self, varargin{:});
    end
    function varargout = computeLinkVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(668, self, varargin{:});
    end
    function varargout = computeJointTorque(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(669, self, varargin{:});
    end
  end
  methods(Static)
  end
end
