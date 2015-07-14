classdef VectorDynSize < iDynTree.IVector
  methods
    function self = VectorDynSize(varargin)
      self@iDynTree.IVector('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(79,'new_VectorDynSize',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(79,'new_VectorDynSize',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(80,'delete_VectorDynSize',self);
        self.swigOwn=false;
      end
    end
    function varargout = TODOparen(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(81,'VectorDynSize_TODOparen',self,varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(82,'VectorDynSize_getVal',self,varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(83,'VectorDynSize_setVal',self,varargin{:});
    end
    function varargout = size(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(84,'VectorDynSize_size',self,varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(85,'VectorDynSize_data',self,varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(86,'VectorDynSize_zero',self,varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(87,'VectorDynSize_resize',self,varargin{:});
    end
    function varargout = fillBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(88,'VectorDynSize_fillBuffer',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(89,'VectorDynSize_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(90,'VectorDynSize_display',self,varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(91,'VectorDynSize_toMatlab',self,varargin{:});
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
