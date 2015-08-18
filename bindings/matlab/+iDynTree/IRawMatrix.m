classdef IRawMatrix < iDynTree.IMatrix
  methods
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(7,'delete_IRawMatrix',self);
        self.swigOwn=false;
      end
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(8,'IRawMatrix_data',self,varargin{:});
    end
    function self = IRawMatrix(varargin)
      self@iDynTree.IMatrix('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
      [v,ok] = swig_fieldsref@iDynTree.IMatrix(self,i);
      if ok
        return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
      [self,ok] = swig_fieldasgn@iDynTree.IMatrix(self,i,v);
      if ok
        return
      end
    end
  end
  methods(Static)
  end
end
