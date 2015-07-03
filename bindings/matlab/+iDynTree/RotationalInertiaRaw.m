classdef RotationalInertiaRaw < SwigRef
  methods
    function self = RotationalInertiaRaw(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(131,'new_RotationalInertiaRaw',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(131,'new_RotationalInertiaRaw',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(132,'delete_RotationalInertiaRaw',self);
        self.swigOwn=false;
      end
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(133,'RotationalInertiaRaw_zero',self,varargin{:});
    end
    function varargout = TODOparen(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(134,'RotationalInertiaRaw_TODOparen',self,varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(135,'RotationalInertiaRaw_getVal',self,varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(136,'RotationalInertiaRaw_setVal',self,varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(137,'RotationalInertiaRaw_rows',self,varargin{:});
    end
    function varargout = cols(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(138,'RotationalInertiaRaw_cols',self,varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(139,'RotationalInertiaRaw_data',self,varargin{:});
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
    end
  end
  methods(Static)
  end
end
