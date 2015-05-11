classdef Twist < iDynTree.SpatialMotionVectorRaw
  methods
    function self = Twist(varargin)
      self@iDynTree.SpatialMotionVectorRaw('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigCPtr = iDynTreeMATLAB_wrap(175,'new_Twist',varargin{:});
        %self.swigOwn = true;
        tmp = iDynTreeMATLAB_wrap(175,'new_Twist',varargin{:}); % FIXME
        self.swigCPtr = tmp.swigCPtr;
        self.swigOwn = tmp.swigOwn;
        self.swigType = tmp.swigType;
        tmp.swigOwn = false;
      end
    end
    function delete(self)
      if self.swigOwn
        iDynTreeMATLAB_wrap(176,'delete_Twist',self);
        self.swigOwn=false;
      end
    end
    function [v,ok] = swig_fieldsref(self,i)
      v = [];
      ok = false;
      switch i
      end
      [v,ok] = swig_fieldsref@iDynTree.SpatialMotionVectorRaw(self,i);
      if ok
        return
      end
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      switch i
      end
      [self,ok] = swig_fieldasgn@iDynTree.SpatialMotionVectorRaw(self,i,v);
      if ok
        return
      end
    end
  end
  methods(Static)
  end
end
