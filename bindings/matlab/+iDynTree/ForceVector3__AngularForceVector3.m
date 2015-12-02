classdef ForceVector3__AngularForceVector3 < iDynTree.GeomVector3__AngularForceVector3
  methods
    function self = ForceVector3__AngularForceVector3(varargin)
      self@iDynTree.GeomVector3__AngularForceVector3('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(283, varargin{:});
        tmp = iDynTreeMATLAB_wrap(283, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(284, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
