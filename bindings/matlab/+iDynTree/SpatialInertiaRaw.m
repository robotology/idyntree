classdef SpatialInertiaRaw < SwigRef
  methods
    function self = SpatialInertiaRaw(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(430, varargin{:});
        tmp = iDynTreeMATLAB_wrap(430, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(431, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = fromRotationalInertiaWrtCenterOfMass(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(432, self, varargin{:});
    end
    function varargout = getMass(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(433, self, varargin{:});
    end
    function varargout = getCenterOfMass(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(434, self, varargin{:});
    end
    function varargout = getRotationalInertiaWrtFrameOrigin(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(435, self, varargin{:});
    end
    function varargout = getRotationalInertiaWrtCenterOfMass(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(436, self, varargin{:});
    end
    function varargout = multiply(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(438, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(439, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = combine(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(437, varargin{:});
    end
  end
end
