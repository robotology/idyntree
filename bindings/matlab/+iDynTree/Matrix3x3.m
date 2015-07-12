classdef Matrix3x3 < iDynTree.IMatrix
  methods
    function self = Matrix3x3(varargin)
      self@iDynTree.IMatrix('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(27,'new_Matrix3x3',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(27,'new_Matrix3x3',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(28,'delete_Matrix3x3',self);
        self.swigOwn=false;
      end
    end
    function varargout = TODOparen(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(29,'Matrix3x3_TODOparen',self,varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(30,'Matrix3x3_getVal',self,varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(31,'Matrix3x3_setVal',self,varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(32,'Matrix3x3_rows',self,varargin{:});
    end
    function varargout = cols(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(33,'Matrix3x3_cols',self,varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(34,'Matrix3x3_data',self,varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(35,'Matrix3x3_zero',self,varargin{:});
    end
    function varargout = fillRowMajorBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(36,'Matrix3x3_fillRowMajorBuffer',self,varargin{:});
    end
    function varargout = fillColMajorBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(37,'Matrix3x3_fillColMajorBuffer',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(38,'Matrix3x3_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(39,'Matrix3x3_display',self,varargin{:});
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
