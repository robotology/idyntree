classdef Link < SwigRef
  methods
    function self = Link(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(381, varargin{:});
        tmp = iDynTreeMATLAB_wrap(381, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(382, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = setInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(383, self, varargin{:});
    end
    function varargout = getInertia(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(384, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(385, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(386, self, varargin{:});
    end
  end
  methods(Static)
  end
end
