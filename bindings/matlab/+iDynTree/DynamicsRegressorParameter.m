classdef DynamicsRegressorParameter < SwigRef
  methods
    function varargout = category(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(727, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(728, self, varargin{1});
      end
    end
    function varargout = elemIndex(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(729, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(730, self, varargin{1});
      end
    end
    function varargout = type(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(731, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(732, self, varargin{1});
      end
    end
    function varargout = lt(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(733, self, varargin{:});
    end
    function varargout = isequal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(734, self, varargin{:});
    end
    function varargout = ne(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(735, self, varargin{:});
    end
    function self = DynamicsRegressorParameter(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(736, varargin{:});
        tmp = iDynTreeMATLAB_wrap(736, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(737, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
