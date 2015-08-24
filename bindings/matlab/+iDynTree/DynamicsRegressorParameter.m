classdef DynamicsRegressorParameter < SwigRef
  methods
    function varargout = category(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(502, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(503, self, varargin{1});
      end
    end
    function varargout = elemIndex(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(504, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(505, self, varargin{1});
      end
    end
    function varargout = type(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(506, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(507, self, varargin{1});
      end
    end
    function varargout = lt(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(508, self, varargin{:});
    end
    function varargout = isequal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(509, self, varargin{:});
    end
    function varargout = ne(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(510, self, varargin{:});
    end
    function self = DynamicsRegressorParameter(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(511, varargin{:});
        tmp = iDynTreeMATLAB_wrap(511, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(512, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
