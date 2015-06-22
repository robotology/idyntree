classdef DynamicsRegressorParametersList < SwigRef
  methods
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(270,'DynamicsRegressorParametersList_getDescriptionOfParameter',self,varargin{:});
    end
    function varargout = addParam(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(271,'DynamicsRegressorParametersList_addParam',self,varargin{:});
    end
    function varargout = addList(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(272,'DynamicsRegressorParametersList_addList',self,varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(273,'DynamicsRegressorParametersList_getNrOfParameters',self,varargin{:});
    end
    function self = DynamicsRegressorParametersList(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(274,'new_DynamicsRegressorParametersList',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(274,'new_DynamicsRegressorParametersList',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(275,'delete_DynamicsRegressorParametersList',self);
        self.swigOwn=false;
      end
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
        case 'parameters'
          v = iDynTreeMATLAB_wrap(268,'DynamicsRegressorParametersList_parameters_get',self);
          ok = true;
          return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
        case 'parameters'
          iDynTreeMATLAB_wrap(269,'DynamicsRegressorParametersList_parameters_set',self,v);
          ok = true;
          return
      end
    end
  end
  methods(Static)
  end
end
