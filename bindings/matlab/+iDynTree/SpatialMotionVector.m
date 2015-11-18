classdef SpatialMotionVector < iDynTree.SpatialMotionVectorBase
  methods
    function self = SpatialMotionVector(varargin)
      self@iDynTree.SpatialMotionVectorBase('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(379, varargin{:});
        tmp = iDynTreeMATLAB_wrap(379, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(380, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(381, self, varargin{:});
    end
    function varargout = cross(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(382, self, varargin{:});
    end
    function varargout = exp(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(383, self, varargin{:});
    end
  end
  methods(Static)
  end
end
