classdef RotationSemantics < SwigRef
  methods
    function self = RotationSemantics(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(137,'new_RotationSemantics',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(137,'new_RotationSemantics',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(138,'delete_RotationSemantics',self);
        self.swigOwn=false;
      end
    end
    function varargout = getOrientationFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(139,'RotationSemantics_getOrientationFrame',self,varargin{:});
    end
    function varargout = getReferenceOrientationFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(140,'RotationSemantics_getReferenceOrientationFrame',self,varargin{:});
    end
    function varargout = setOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(141,'RotationSemantics_setOrientationFrame',self,varargin{:});
    end
    function varargout = setReferenceOrientationFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(142,'RotationSemantics_setReferenceOrientationFrame',self,varargin{:});
    end
    function varargout = check_changeOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(143,'RotationSemantics_check_changeOrientFrame',self,varargin{:});
    end
    function varargout = check_changeRefOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(144,'RotationSemantics_check_changeRefOrientFrame',self,varargin{:});
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(148,'RotationSemantics_changeOrientFrame',self,varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(149,'RotationSemantics_changeRefOrientFrame',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(153,'RotationSemantics_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(154,'RotationSemantics_display',self,varargin{:});
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
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(145,'RotationSemantics_check_compose',varargin{:});
    end
    function varargout = check_inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(146,'RotationSemantics_check_inverse2',varargin{:});
    end
    function varargout = check_transform(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(147,'RotationSemantics_check_transform',varargin{:});
    end
    function varargout = compose(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(150,'RotationSemantics_compose',varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(151,'RotationSemantics_inverse2',varargin{:});
    end
    function varargout = transform(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(152,'RotationSemantics_transform',varargin{:});
    end
  end
end
