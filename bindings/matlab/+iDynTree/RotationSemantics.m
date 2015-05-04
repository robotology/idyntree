classdef RotationSemantics < SwigRef
  methods
    function self = RotationSemantics(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(53,'new_RotationSemantics',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(53,'new_RotationSemantics',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(54,'delete_RotationSemantics',self);
        self.swigOwn=false;
      end
    end
    function varargout = getOrientationFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(55,'RotationSemantics_getOrientationFrame',self,varargin{:});
    end
    function varargout = getReferenceOrientationFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(56,'RotationSemantics_getReferenceOrientationFrame',self,varargin{:});
    end
    function varargout = setOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(57,'RotationSemantics_setOrientationFrame',self,varargin{:});
    end
    function varargout = setReferenceOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(58,'RotationSemantics_setReferenceOrientationFrame',self,varargin{:});
    end
    function varargout = check_changeOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(59,'RotationSemantics_check_changeOrientFrame',self,varargin{:});
    end
    function varargout = check_changeRefOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(60,'RotationSemantics_check_changeRefOrientFrame',self,varargin{:});
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(64,'RotationSemantics_changeOrientFrame',self,varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(65,'RotationSemantics_changeRefOrientFrame',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(69,'RotationSemantics_toString',self,varargin{:});
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
    function varargout = check_compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(61,'RotationSemantics_check_compose',varargin{:});
    end
    function varargout = check_inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(62,'RotationSemantics_check_inverse2',varargin{:});
    end
    function varargout = check_apply(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(63,'RotationSemantics_check_apply',varargin{:});
    end
    function varargout = compose(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(66,'RotationSemantics_compose',varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(67,'RotationSemantics_inverse2',varargin{:});
    end
    function varargout = apply(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(68,'RotationSemantics_apply',varargin{:});
    end
  end
end
