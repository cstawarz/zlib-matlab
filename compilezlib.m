function compilezlib(varargin)
    mex(varargin{:}, '-lz', 'zlib.cpp')
end
