classdef Transform < SwigRef
  methods
    function self = Transform(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(502, varargin{:});
        tmp = iDynTreeMATLAB_wrap(502, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(503, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(504, self, varargin{:});
    end
    function varargout = getRotation(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(505, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(506, self, varargin{:});
    end
    function varargout = setRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(507, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(508, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(511, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(512, self, varargin{:});
    end
    function varargout = asHomogeneousTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(514, self, varargin{:});
    end
    function varargout = asAdjointTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(515, self, varargin{:});
    end
    function varargout = asAdjointTransformWrench(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(516, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(517, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(518, self, varargin{:});
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(509, varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(510, varargin{:});
    end
    function varargout = Identity(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(513, varargin{:});
    end
  end
end
