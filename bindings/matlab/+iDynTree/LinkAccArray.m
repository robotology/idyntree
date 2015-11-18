classdef LinkAccArray < SwigRef
  methods
    function self = LinkAccArray(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(570, varargin{:});
        tmp = iDynTreeMATLAB_wrap(570, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(571, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(572, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(573, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(574, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
