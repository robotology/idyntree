classdef ConvertSem2motionForceTraits__AngularForceVector3Semantics < SwigRef
  methods
    function self = ConvertSem2motionForceTraits__AngularForceVector3Semantics(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(218, varargin{:});
        tmp = iDynTreeMATLAB_wrap(218, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(219, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
