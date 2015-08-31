classdef Axis < SwigRef
  methods
    function self = Axis(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(451, varargin{:});
        tmp = iDynTreeMATLAB_wrap(451, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(452, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(453, self, varargin{:});
    end
    function varargout = getOrigin(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(454, self, varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(455, self, varargin{:});
    end
    function varargout = setOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(456, self, varargin{:});
    end
    function varargout = getRotationTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(457, self, varargin{:});
    end
    function varargout = getRotationTwist(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(458, self, varargin{:});
    end
    function varargout = getRotationSpatialAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(459, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(460, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(461, self, varargin{:});
    end
  end
  methods(Static)
  end
end
