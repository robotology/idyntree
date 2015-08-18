classdef IRawVector < iDynTree.IVector
  methods
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(14,'delete_IRawVector',self);
        self.swigOwn=false;
      end
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(15,'IRawVector_data',self,varargin{:});
    end
    function self = IRawVector(varargin)
      self@iDynTree.IVector('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
      [v,ok] = swig_fieldsref@iDynTree.IVector(self,i);
      if ok
        return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
      [self,ok] = swig_fieldasgn@iDynTree.IVector(self,i,v);
      if ok
        return
      end
    end
  end
  methods(Static)
  end
end
