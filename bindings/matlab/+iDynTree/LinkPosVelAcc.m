classdef LinkPosVelAcc < SwigRef
  methods
    function varargout = pos(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(539, self, varargin{:});
    end
    function varargout = vel(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(540, self, varargin{:});
    end
    function varargout = acc(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(541, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(542, self);
        self.swigInd=uint64(0);
      end
    end
    function self = LinkPosVelAcc(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(543, varargin{:});
        tmp = iDynTreeMATLAB_wrap(543, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
  end
  methods(Static)
  end
end
