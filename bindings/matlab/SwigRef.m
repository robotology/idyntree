classdef SwigRef < handle
  properties % (GetAccess = protected, SetAccess = protected) % FIXME: mxGetProperty not working with protected access 
    swigCPtr
    swigType
    swigOwn
  end
  methods
    function disp(self)
      disp(sprintf('<Swig Object of type %s, ptr=%d, own=%d>',self.swigType,self.swigCPtr,self.swigOwn))
    end
    function varargout = subsref(self,S)
      if numel(S)==1
        if S.type=='.'
          [varargout{1},ok] = swig_fieldsref(self,S.subs);
          if ok
            return
          end
        elseif S.type=='()'
          varargout{1} = getitem(self,S.subs{:});
          return;
        else
          varargout{1} = getitemcurl(self,S.subs{:});
          return;
        end
      end
      [varargout{1:nargout}] = builtin('subsref',self,S);
    end
    function [v,ok] = swig_fieldsref(self,subs)
      v = [];
      ok = false;
    end
    function self = subsasgn(self,S,v)
      if numel(S)==1
        if S.type=='.'
          [self,ok] = swig_fieldasgn(self,S.subs,v);
          if ok
            return
          end
        elseif S.type=='()'
          setitem(self,v,S.subs{:});
          return;
        else
          setitemcurl(self,v,S.subs{:});
          return;
        end
      end
      self = builtin('subsasgn',self,S,v);
    end
    function [self,ok] = swig_fieldasgn(self,i,v)
      ok = false;
    end
  end
end
