classdef SpatialMotionVector < iDynTree.SpatialMotionVectorBase
  methods
    function self = SpatialMotionVector(varargin)
      self@iDynTree.SpatialMotionVectorBase('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(420, varargin{:});
        tmp = iDynTreeMATLAB_wrap(420, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(421, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = cross(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(422, self, varargin{:});
    end
  end
  methods(Static)
  end
end
