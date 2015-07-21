classdef RotationSemantics < SwigRef
  methods
    function self = RotationSemantics(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(228,'new_RotationSemantics',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(228,'new_RotationSemantics',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(229,'delete_RotationSemantics',self);
        self.swigOwn=false;
      end
    end
    function varargout = getOrientationFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(230,'RotationSemantics_getOrientationFrame',self,varargin{:});
    end
    function varargout = getReferenceOrientationFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(231,'RotationSemantics_getReferenceOrientationFrame',self,varargin{:});
    end
    function varargout = getCoordinateFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(232,'RotationSemantics_getCoordinateFrame',self,varargin{:});
    end
    function varargout = setOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(233,'RotationSemantics_setOrientationFrame',self,varargin{:});
    end
    function varargout = setReferenceOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(234,'RotationSemantics_setReferenceOrientationFrame',self,varargin{:});
    end
    function varargout = setCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(235,'RotationSemantics_setCoordinateFrame',self,varargin{:});
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(236,'RotationSemantics_changeOrientFrame',self,varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(237,'RotationSemantics_changeRefOrientFrame',self,varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(238,'RotationSemantics_changeCoordFrameOf',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(241,'RotationSemantics_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(242,'RotationSemantics_display',self,varargin{:});
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
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(239,'RotationSemantics_compose',varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(240,'RotationSemantics_inverse2',varargin{:});
    end
  end
end
