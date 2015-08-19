classdef ClassicalAcc < iDynTree.Vector6
  methods
    function self = ClassicalAcc(varargin)
      self@iDynTree.Vector6('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(216,'new_ClassicalAcc',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(216,'new_ClassicalAcc',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(217,'delete_ClassicalAcc',self);
        self.swigOwn=false;
      end
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(218,'ClassicalAcc_changeCoordFrame',self,varargin{:});
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
      [v,ok] = swig_fieldsref@iDynTree.Vector6(self,i);
      if ok
        return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
      [self,ok] = swig_fieldasgn@iDynTree.Vector6(self,i,v);
      if ok
        return
      end
    end
  end
  methods(Static)
    function varargout = Zero(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(219,'ClassicalAcc_Zero',varargin{:});
    end
  end
end
