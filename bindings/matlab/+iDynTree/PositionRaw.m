classdef PositionRaw < iDynTree.Vector3
  methods
    function self = PositionRaw(varargin)
      self@iDynTree.Vector3('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(136,'new_PositionRaw',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(136,'new_PositionRaw',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(137,'delete_PositionRaw',self);
        self.swigOwn=false;
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(138,'PositionRaw_changePoint',self,varargin{:});
    end
    function varargout = changeRefPoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(139,'PositionRaw_changeRefPoint',self,varargin{:});
    end
    function varargout = changePointOf(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(142,'PositionRaw_changePointOf',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(143,'PositionRaw_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(144,'PositionRaw_display',self,varargin{:});
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
      [v,ok] = swig_fieldsref@iDynTree.Vector3(self,i);
      if ok
        return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
      [self,ok] = swig_fieldasgn@iDynTree.Vector3(self,i,v);
      if ok
        return
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(140,'PositionRaw_compose',varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(141,'PositionRaw_inverse',varargin{:});
    end
  end
end
