classdef LinkVelAccArray < SwigRef
  methods
    function self = LinkVelAccArray(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(548, varargin{:});
        tmp = iDynTreeMATLAB_wrap(548, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(549, self, varargin{:});
    end
    function varargout = linkVelAcc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(550, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(551, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
