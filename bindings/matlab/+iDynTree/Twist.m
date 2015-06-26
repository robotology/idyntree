classdef Twist < iDynTree.SpatialMotionVectorRaw
  methods
    function self = Twist(varargin)
      self@iDynTree.SpatialMotionVectorRaw('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(119,'new_Twist',varargin{:});
        tmp = iDynTreeMATLAB_wrap(119,'new_Twist',varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(120,'delete_Twist',self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
