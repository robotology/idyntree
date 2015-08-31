classdef LinkPos < SwigRef
  methods
    function varargout = pos(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(565, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(566, self);
        self.swigInd=uint64(0);
      end
    end
    function self = LinkPos(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(567, varargin{:});
        tmp = iDynTreeMATLAB_wrap(567, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
  end
  methods(Static)
  end
end
