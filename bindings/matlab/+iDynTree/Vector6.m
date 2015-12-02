classdef Vector6 < SwigRef
  methods
    function self = Vector6(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(105, varargin{:});
        tmp = iDynTreeMATLAB_wrap(105, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = paren(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(106, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(107, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(108, self, varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(109, self, varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(110, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(111, self, varargin{:});
    end
    function varargout = fillBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(112, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(113, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(114, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(115, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(116, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(117, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
