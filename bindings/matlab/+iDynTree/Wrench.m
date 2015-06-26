classdef Wrench < iDynTree.SpatialForceVectorRaw
  methods
    function self = Wrench(varargin)
      self@iDynTree.SpatialForceVectorRaw('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(121,'new_Wrench',varargin{:});
        tmp = iDynTreeMATLAB_wrap(121,'new_Wrench',varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(122,'delete_Wrench',self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
