classdef DynamicsRegressorParameter < SwigRef
  methods
    function varargout = category(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(263, 'DynamicsRegressorParameter_category_get', self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(264, 'DynamicsRegressorParameter_category_set', self, varargin{1});
      end
    end
    function varargout = elemIndex(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(265, 'DynamicsRegressorParameter_elemIndex_get', self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(266, 'DynamicsRegressorParameter_elemIndex_set', self, varargin{1});
      end
    end
    function varargout = type(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(267, 'DynamicsRegressorParameter_type_get', self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(268, 'DynamicsRegressorParameter_type_set', self, varargin{1});
      end
    end
    function varargout = lt(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(269,'DynamicsRegressorParameter_lt',self,varargin{:});
    end
    function varargout = isequal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(270,'DynamicsRegressorParameter_isequal',self,varargin{:});
    end
    function varargout = ne(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(271,'DynamicsRegressorParameter_ne',self,varargin{:});
    end
    function self = DynamicsRegressorParameter(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(272,'new_DynamicsRegressorParameter',varargin{:});
        tmp = iDynTreeMATLAB_wrap(272,'new_DynamicsRegressorParameter',varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(273,'delete_DynamicsRegressorParameter',self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
