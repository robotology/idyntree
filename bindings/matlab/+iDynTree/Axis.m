classdef Axis < SwigRef
  methods
    function self = Axis(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(245, varargin{:});
        tmp = iDynTreeMATLAB_wrap(245, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(246, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(247, self, varargin{:});
    end
    function varargout = getOrigin(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(248, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(249, self, varargin{:});
    end
    function varargout = setOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(250, self, varargin{:});
    end
    function varargout = getRotationTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(251, self, varargin{:});
    end
    function varargout = getRotationTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(252, self, varargin{:});
    end
    function varargout = getRotationSpatialAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(253, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(254, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(255, self, varargin{:});
    end
  end
  methods(Static)
  end
end
