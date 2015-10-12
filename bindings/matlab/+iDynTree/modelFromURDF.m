function varargout = modelFromURDF(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(660,varargin{:});
end
