classdef DynamicsRegressorParameter < SwigRef
  methods
    function varargout = lt(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(264,'DynamicsRegressorParameter_lt',self,varargin{:});
    end
    function varargout = isequal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(265,'DynamicsRegressorParameter_isequal',self,varargin{:});
    end
    function self = DynamicsRegressorParameter(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(266,'new_DynamicsRegressorParameter',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(266,'new_DynamicsRegressorParameter',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(267,'delete_DynamicsRegressorParameter',self);
        self.swigOwn=false;
      end
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
        case 'category'
          v = iDynTreeMATLAB_wrap(258,'DynamicsRegressorParameter_category_get',self);
          ok = true;
          return
        case 'index'
          v = iDynTreeMATLAB_wrap(260,'DynamicsRegressorParameter_index_get',self);
          ok = true;
          return
        case 'type'
          v = iDynTreeMATLAB_wrap(262,'DynamicsRegressorParameter_type_get',self);
          ok = true;
          return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
        case 'category'
          iDynTreeMATLAB_wrap(259,'DynamicsRegressorParameter_category_set',self,v);
          ok = true;
          return
        case 'index'
          iDynTreeMATLAB_wrap(261,'DynamicsRegressorParameter_index_set',self,v);
          ok = true;
          return
        case 'type'
          iDynTreeMATLAB_wrap(263,'DynamicsRegressorParameter_type_set',self,v);
          ok = true;
          return
      end
    end
  end
  methods(Static)
  end
end
