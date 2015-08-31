classdef IJointPos < SwigRef
  methods
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(577, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = pos(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(578, self, varargin{:});
    end
    function varargout = getNrOfPosCoords(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(579, self, varargin{:});
    end
    function self = IJointPos(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
