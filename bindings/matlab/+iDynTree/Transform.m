classdef Transform < SwigRef
  methods
    function self = Transform(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(209,'new_Transform',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(209,'new_Transform',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(210,'delete_Transform',self);
        self.swigOwn=false;
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(211,'Transform_getSemantics',self,varargin{:});
    end
    function varargout = getRotation(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(212,'Transform_getRotation',self,varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(213,'Transform_getPosition',self,varargin{:});
    end
    function varargout = setRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(214,'Transform_setRotation',self,varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(215,'Transform_setPosition',self,varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(219,'Transform_inverse',self,varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(220,'Transform_mtimes',self,varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(222,'Transform_toString',self,varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(223,'Transform_display',self,varargin{:});
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
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(216,'Transform_compose',varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(217,'Transform_inverse2',varargin{:});
    end
    function varargout = transform(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(218,'Transform_transform',varargin{:});
    end
    function varargout = Identity(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(221,'Transform_Identity',varargin{:});
    end
  end
end
