classdef MotionVector3__AngularMotionVector3 < iDynTree.GeomVector3__AngularMotionVector3
  methods
    function self = MotionVector3__AngularMotionVector3(varargin)
      self@iDynTree.GeomVector3__AngularMotionVector3('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(287, varargin{:});
        tmp = iDynTreeMATLAB_wrap(287, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(288, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = cross(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(289, self, varargin{:});
    end
  end
  methods(Static)
  end
end
