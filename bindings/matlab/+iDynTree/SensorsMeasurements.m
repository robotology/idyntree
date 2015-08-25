classdef SensorsMeasurements < SwigRef
  methods
    function self = SensorsMeasurements(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(470, varargin{:});
        tmp = iDynTreeMATLAB_wrap(470, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(471, self);
        self.swigInd=uint64(0);
      end
    end
    function varargout = setNrOfSensors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(472, self, varargin{:});
    end
    function varargout = getNrOfSensors(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(473, self, varargin{:});
    end
    function varargout = setMeasurement(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(474, self, varargin{:});
    end
    function varargout = getMeasurement(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(475, self, varargin{:});
    end
  end
  methods(Static)
  end
end
