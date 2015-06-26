classdef SpatialForceVectorRaw < iDynTree.Vector6
  methods
    function self = SpatialForceVectorRaw(varargin)
      self@iDynTree.Vector6('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(97,'new_SpatialForceVectorRaw',varargin{:});
        tmp = iDynTreeMATLAB_wrap(97,'new_SpatialForceVectorRaw',varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(98,'delete_SpatialForceVectorRaw',self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(99,'SpatialForceVectorRaw_changePoint',self,varargin{:});
    end
    function varargout = changeCoordFrame(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(100,'SpatialForceVectorRaw_changeCoordFrame',self,varargin{:});
    end
    function varargout = dot(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(103,'SpatialForceVectorRaw_dot',self,varargin{:});
    end
    function varargout = plus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(104,'SpatialForceVectorRaw_plus',self,varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(105,'SpatialForceVectorRaw_minus',self,varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(106,'SpatialForceVectorRaw_uminus',self,varargin{:});
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(101,'SpatialForceVectorRaw_compose',varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(102,'SpatialForceVectorRaw_inverse',varargin{:});
    end
    function varargout = Zero(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(107,'SpatialForceVectorRaw_Zero',varargin{:});
    end
  end
end
