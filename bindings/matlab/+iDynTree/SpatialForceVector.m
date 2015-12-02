classdef SpatialForceVector < iDynTree.SpatialForceVectorBase
  methods
    function self = SpatialForceVector(varargin)
      self@iDynTree.SpatialForceVectorBase('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(377, varargin{:});
        tmp = iDynTreeMATLAB_wrap(377, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(378, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(379, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
