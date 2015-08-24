classdef Matrix6x6 < iDynTree.IRawMatrix
  methods
    function self = Matrix6x6(varargin)
      self@iDynTree.IRawMatrix('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(77, varargin{:});
        tmp = iDynTreeMATLAB_wrap(77, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(78, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = paren(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(79, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(80, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(81, self, varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(82, self, varargin{:});
    end
    function varargout = cols(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(83, self, varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(84, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(85, self, varargin{:});
    end
    function varargout = fillRowMajorBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(86, self, varargin{:});
    end
    function varargout = fillColMajorBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(87, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(88, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(89, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(90, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(91, self, varargin{:});
    end
  end
  methods(Static)
  end
end
