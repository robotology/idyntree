classdef Axis < SwigRef
  methods
    function self = Axis(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(225,'new_Axis',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(225,'new_Axis',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(226,'delete_Axis',self);
        self.swigOwn=false;
      end
    end
    function varargout = getDirection(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(227,'Axis_getDirection',self,varargin{:});
    end
    function varargout = getOrigin(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(228,'Axis_getOrigin',self,varargin{:});
    end
    function varargout = setDirection(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(229,'Axis_setDirection',self,varargin{:});
    end
    function varargout = setOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(230,'Axis_setOrigin',self,varargin{:});
    end
    function varargout = getRotationTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(231,'Axis_getRotationTransform',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(232,'Axis_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(233,'Axis_display',self,varargin{:});
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
