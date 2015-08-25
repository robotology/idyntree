classdef Link < SwigRef
  methods
    function self = Link(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(362, varargin{:});
        tmp = iDynTreeMATLAB_wrap(362, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(363, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = setInertia(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(364, self, varargin{:});
    end
    function varargout = getInertia(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(365, self, varargin{:});
    end
    function varargout = setIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(366, self, varargin{:});
    end
    function varargout = getIndex(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(367, self, varargin{:});
    end
  end
  methods(Static)
  end
end
