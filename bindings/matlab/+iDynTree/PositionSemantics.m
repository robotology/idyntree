classdef PositionSemantics < SwigRef
  methods
    function self = PositionSemantics(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(67,'new_PositionSemantics',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(67,'new_PositionSemantics',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(68,'delete_PositionSemantics',self);
        self.swigOwn=false;
      end
    end
    function varargout = getPoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(69,'PositionSemantics_getPoint',self,varargin{:});
    end
    function varargout = getReferencePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(70,'PositionSemantics_getReferencePoint',self,varargin{:});
    end
    function varargout = getCoordinateFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(71,'PositionSemantics_getCoordinateFrame',self,varargin{:});
    end
    function varargout = setPoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(72,'PositionSemantics_setPoint',self,varargin{:});
    end
    function varargout = setReferencePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(73,'PositionSemantics_setReferencePoint',self,varargin{:});
    end
    function varargout = setCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(74,'PositionSemantics_setCoordinateFrame',self,varargin{:});
    end
    function varargout = check_changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(75,'PositionSemantics_check_changePoint',self,varargin{:});
    end
    function varargout = check_changeRefPoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(76,'PositionSemantics_check_changeRefPoint',self,varargin{:});
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(79,'PositionSemantics_changePoint',self,varargin{:});
    end
    function varargout = changeRefPoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(80,'PositionSemantics_changeRefPoint',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(83,'PositionSemantics_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(84,'PositionSemantics_display',self,varargin{:});
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
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(77,'PositionSemantics_check_compose',varargin{:});
    end
    function varargout = check_inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(78,'PositionSemantics_check_inverse',varargin{:});
    end
    function varargout = compose(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(81,'PositionSemantics_compose',varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(82,'PositionSemantics_inverse',varargin{:});
    end
  end
end
