classdef RotationRaw < iDynTree.Matrix3x3
  methods
    function self = RotationRaw(varargin)
      self@iDynTree.Matrix3x3('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(250,'new_RotationRaw',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(250,'new_RotationRaw',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(251,'delete_RotationRaw',self);
        self.swigOwn=false;
      end
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(252,'RotationRaw_changeOrientFrame',self,varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(253,'RotationRaw_changeRefOrientFrame',self,varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(256,'RotationRaw_changeCoordFrameOf',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(262,'RotationRaw_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(263,'RotationRaw_display',self,varargin{:});
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
      [v,ok] = swig_fieldsref@iDynTree.Matrix3x3(self,i);
      if ok
        return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
      [self,ok] = swig_fieldasgn@iDynTree.Matrix3x3(self,i,v);
      if ok
        return
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(254,'RotationRaw_compose',varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(255,'RotationRaw_inverse2',varargin{:});
    end
    function varargout = RotX(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(257,'RotationRaw_RotX',varargin{:});
    end
    function varargout = RotY(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(258,'RotationRaw_RotY',varargin{:});
    end
    function varargout = RotZ(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(259,'RotationRaw_RotZ',varargin{:});
    end
    function varargout = RPY(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(260,'RotationRaw_RPY',varargin{:});
    end
    function varargout = Identity(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(261,'RotationRaw_Identity',varargin{:});
    end
  end
end
