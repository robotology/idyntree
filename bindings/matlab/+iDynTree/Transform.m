classdef Transform < SwigRef
  methods
    function self = Transform(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(302,'new_Transform',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(302,'new_Transform',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(303,'delete_Transform',self);
        self.swigOwn=false;
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(304,'Transform_getSemantics',self,varargin{:});
    end
    function varargout = getRotation(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(305,'Transform_getRotation',self,varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(306,'Transform_getPosition',self,varargin{:});
    end
    function varargout = setRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(307,'Transform_setRotation',self,varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(308,'Transform_setPosition',self,varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(311,'Transform_inverse',self,varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(312,'Transform_mtimes',self,varargin{:});
    end
    function varargout = asHomogeneousTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(314,'Transform_asHomogeneousTransform',self,varargin{:});
    end
    function varargout = asAdjointTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(315,'Transform_asAdjointTransform',self,varargin{:});
    end
    function varargout = asAdjointTransformWrench(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(316,'Transform_asAdjointTransformWrench',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(317,'Transform_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(318,'Transform_display',self,varargin{:});
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
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(309,'Transform_compose',varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(310,'Transform_inverse2',varargin{:});
    end
    function varargout = Identity(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(313,'Transform_Identity',varargin{:});
    end
  end
end
