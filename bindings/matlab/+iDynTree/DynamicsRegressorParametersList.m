classdef DynamicsRegressorParametersList < SwigRef
  methods
    function varargout = parameters(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMATLAB_wrap(790, self);
      else
        nargoutchk(0, 0)
        iDynTreeMATLAB_wrap(791, self, varargin{1});
      end
    end
    function varargout = getDescriptionOfParameter(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(792, self, varargin{:});
    end
    function varargout = addParam(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(793, self, varargin{:});
    end
    function varargout = addList(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(794, self, varargin{:});
    end
    function varargout = findParam(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(795, self, varargin{:});
    end
    function varargout = getNrOfParameters(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(796, self, varargin{:});
    end
    function self = DynamicsRegressorParametersList(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(797, varargin{:});
        tmp = iDynTreeMATLAB_wrap(797, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(798, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
