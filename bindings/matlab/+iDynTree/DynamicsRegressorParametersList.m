classdef DynamicsRegressorParametersList < SwigRef
  methods
    function varargout = parameters(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(274, 'DynamicsRegressorParametersList_parameters_get', self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(275, 'DynamicsRegressorParametersList_parameters_set', self, varargin{1});
      end
    end
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(276,'DynamicsRegressorParametersList_getDescriptionOfParameter',self,varargin{:});
    end
    function varargout = addParam(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(277,'DynamicsRegressorParametersList_addParam',self,varargin{:});
    end
    function varargout = addList(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(278,'DynamicsRegressorParametersList_addList',self,varargin{:});
    end
    function varargout = findParam(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(279,'DynamicsRegressorParametersList_findParam',self,varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(280,'DynamicsRegressorParametersList_getNrOfParameters',self,varargin{:});
    end
    function self = DynamicsRegressorParametersList(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(281,'new_DynamicsRegressorParametersList',varargin{:});
        tmp = iDynTreeMATLAB_wrap(281,'new_DynamicsRegressorParametersList',varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(282,'delete_DynamicsRegressorParametersList',self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
