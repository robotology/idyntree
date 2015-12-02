classdef ArticulatedBodyInertia < SwigRef
  methods
    function self = ArticulatedBodyInertia(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(443, varargin{:});
        tmp = iDynTreeMATLAB_wrap(443, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = getLinearLinearSubmatrix(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(444, self, varargin{:});
    end
    function varargout = getLinearAngularSubmatrix(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(445, self, varargin{:});
    end
    function varargout = getAngularAngularSubmatrix(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(446, self, varargin{:});
    end
    function varargout = applyInverse(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(448, self, varargin{:});
    end
    function varargout = asMatrix(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(449, self, varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(450, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(451, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(452, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(453, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(455, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
    function varargout = combine(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(447, varargin{:});
    end
    function varargout = ABADyadHelper(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(454, varargin{:});
    end
  end
end
