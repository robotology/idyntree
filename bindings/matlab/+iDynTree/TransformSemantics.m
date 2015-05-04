classdef TransformSemantics < SwigRef
  methods
    function self = TransformSemantics(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(89,'new_TransformSemantics',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(89,'new_TransformSemantics',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(90,'delete_TransformSemantics',self);
        self.swigOwn=false;
      end
    end
    function varargout = getRotationSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(91,'TransformSemantics_getRotationSemantics',self,varargin{:});
    end
    function varargout = getPositionSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(92,'TransformSemantics_getPositionSemantics',self,varargin{:});
    end
    function varargout = setRotationSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(93,'TransformSemantics_setRotationSemantics',self,varargin{:});
    end
    function varargout = setPositionSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(94,'TransformSemantics_setPositionSemantics',self,varargin{:});
    end
    function varargout = getPoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(95,'TransformSemantics_getPoint',self,varargin{:});
    end
    function varargout = getOrientationFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(96,'TransformSemantics_getOrientationFrame',self,varargin{:});
    end
    function varargout = getReferencePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(97,'TransformSemantics_getReferencePoint',self,varargin{:});
    end
    function varargout = getReferenceOrientationFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(98,'TransformSemantics_getReferenceOrientationFrame',self,varargin{:});
    end
    function varargout = setPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(99,'TransformSemantics_setPoint',self,varargin{:});
    end
    function varargout = setOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(100,'TransformSemantics_setOrientationFrame',self,varargin{:});
    end
    function varargout = setReferencePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(101,'TransformSemantics_setReferencePoint',self,varargin{:});
    end
    function varargout = setReferenceOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(102,'TransformSemantics_setReferenceOrientationFrame',self,varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(109,'TransformSemantics_inverse',self,varargin{:});
    end
    function varargout = mult(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(110,'TransformSemantics_mult',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(111,'TransformSemantics_toString',self,varargin{:});
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
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(103,'TransformSemantics_check_compose',varargin{:});
    end
    function varargout = check_inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(104,'TransformSemantics_check_inverse2',varargin{:});
    end
    function varargout = check_apply(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(105,'TransformSemantics_check_apply',varargin{:});
    end
    function varargout = compose(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(106,'TransformSemantics_compose',varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(107,'TransformSemantics_inverse2',varargin{:});
    end
    function varargout = apply(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(108,'TransformSemantics_apply',varargin{:});
    end
  end
end
