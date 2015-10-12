classdef Neighbor < SwigRef
  methods
    function varargout = neighborLink(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(639, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(640, self, varargin{1});
      end
    end
    function varargout = neighborJoint(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(641, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(642, self, varargin{1});
      end
    end
    function self = Neighbor(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(643, varargin{:});
        tmp = iDynTreeMATLAB_wrap(643, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(644, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
