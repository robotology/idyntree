classdef AngularForceVector3 < SwigRef
  methods
    function self = AngularForceVector3(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(207, varargin{:});
        tmp = iDynTreeMATLAB_wrap(207, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(208, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(209, self, varargin{:});
    end
  end
  methods(Static)
  end
end
