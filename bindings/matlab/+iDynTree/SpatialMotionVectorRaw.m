classdef SpatialMotionVectorRaw < iDynTree.Vector6
  methods
    function self = SpatialMotionVectorRaw(varargin)
      self@iDynTree.Vector6('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(175,'new_SpatialMotionVectorRaw',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(175,'new_SpatialMotionVectorRaw',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(176,'delete_SpatialMotionVectorRaw',self);
        self.swigOwn=false;
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(177,'SpatialMotionVectorRaw_changePoint',self,varargin{:});
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(178,'SpatialMotionVectorRaw_changeCoordFrame',self,varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(181,'SpatialMotionVectorRaw_dot',self,varargin{:});
    end
    function varargout = cross(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(182,'SpatialMotionVectorRaw_cross',self,varargin{:});
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
      [v,ok] = swig_fieldsref@iDynTree.Vector6(self,i);
      if ok
        return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
      [self,ok] = swig_fieldasgn@iDynTree.Vector6(self,i,v);
      if ok
        return
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(179,'SpatialMotionVectorRaw_compose',varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(180,'SpatialMotionVectorRaw_inverse',varargin{:});
    end
    function varargout = Zero(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(183,'SpatialMotionVectorRaw_Zero',varargin{:});
    end
  end
end
