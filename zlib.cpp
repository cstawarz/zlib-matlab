#include <string>

#include <zlib.h>

#include "mex.h"

#define ZLIB_MATLAB_NAMESPACE_BEGIN namespace {
#define ZLIB_MATLAB_NAMESPACE_END   }

#define ZLIB_MATLAB_STRING(name, value) const char * const name = value
#define ZLIB_MATLAB_ERR_ID(name, value) ZLIB_MATLAB_STRING(name, "ZLib:" value)


ZLIB_MATLAB_NAMESPACE_BEGIN


ZLIB_MATLAB_STRING(COMPRESS_ACTION, "compress");
ZLIB_MATLAB_STRING(DECOMPRESS_ACTION, "decompress");

ZLIB_MATLAB_ERR_ID(WRONG_NUM_OUTPUTS, "WrongNumberOfOutputs");
ZLIB_MATLAB_ERR_ID(WRONG_NUM_INPUTS, "WrongNumberOfInputs");
ZLIB_MATLAB_ERR_ID(INVALID_INPUT, "InvalidInput");
ZLIB_MATLAB_ERR_ID(OUT_OF_MEMORY, "OutOfMemory");
ZLIB_MATLAB_ERR_ID(BUFFER_EXHAUSTED, "BufferExhausted");
ZLIB_MATLAB_ERR_ID(WRONG_LIBRARY_VERSION, "WrongLibraryVersion");
ZLIB_MATLAB_ERR_ID(UNKNOWN_ERROR, "UnknownError");


std::string getString(const mxArray *inputs[], int index) {
    auto input = inputs[index];
    char *str = nullptr;
    if (!mxIsChar(input) || !(str = mxArrayToString(input))) {
        mexErrMsgIdAndTxt(INVALID_INPUT,
                          "Input %d must be a character array",
                          index+1);
    }
    auto result = std::string(str);
    mxFree(str);
    return result;
}


std::size_t getInputData(const mxArray *inputs[], int index, z_stream &stream) {
    auto input = inputs[index];
    if (mxGetClassID(input) != mxUINT8_CLASS || mxIsComplex(input)) {
        mexErrMsgIdAndTxt(INVALID_INPUT,
                          "Input %d must be a noncomplex uint8 array",
                          index+1);
    }
#if MX_HAS_INTERLEAVED_COMPLEX
    auto data = mxGetUint8s(input);
#else
    auto data = static_cast<std::uint8_t *>(mxGetData(input));
#endif
    auto size = mxGetNumberOfElements(input);
    stream.next_in = data;
    stream.avail_in = size;
    return size;
}


int getInteger(const mxArray *inputs[], int index) {
    auto input = inputs[index];
    if (!mxIsNumeric(input) || !mxIsScalar(input) || mxIsComplex(input)) {
        mexErrMsgIdAndTxt(INVALID_INPUT,
                          "Input %d must be a noncomplex numeric scalar",
                          index+1);
    }
    auto value = mxGetScalar(input);
    auto result = int(value);
    if (double(result) != value) {
        mexErrMsgIdAndTxt(INVALID_INPUT,
                          "Input %d must be an integer",
                          index+1);
    }
    return result;
}


std::uint8_t * createOutputData(std::size_t size, z_stream &stream) {
    auto data = static_cast<std::uint8_t *>(mxMalloc(size));
    stream.next_out = data;
    stream.avail_out = size;
    return data;
}


mxArray * createOutput(std::uint8_t *data,
                       std::size_t size,
                       const z_stream &stream)
{
    auto bytesUsed = size - stream.avail_out;
    if (bytesUsed < size) {
        size = bytesUsed;
        data = static_cast<std::uint8_t *>(mxRealloc(data, size));
    }

    auto output = mxCreateNumericMatrix(0, 0, mxUINT8_CLASS, mxREAL);
#if MX_HAS_INTERLEAVED_COMPLEX
    mxSetUint8s(output, data);
#else
    mxSetData(output, data);
#endif
    mxSetM(output, 1);
    mxSetN(output, size);

    return output;
}


void throwZLibError(int rc, const z_stream &stream) {
    const char *errID = nullptr;
    std::string msg;

    switch (rc) {
    case Z_STREAM_ERROR:
        errID = INVALID_INPUT;
        msg = "Invalid input";
        break;
    case Z_DATA_ERROR:
        errID = INVALID_INPUT;
        msg = "Input data is corrupted";
        break;
    case Z_MEM_ERROR:
        errID = OUT_OF_MEMORY;
        msg = "Out of memory";
        break;
    case Z_BUF_ERROR:
        errID = BUFFER_EXHAUSTED;
        msg = "Buffer exhausted";
        break;
    case Z_VERSION_ERROR:
        errID = WRONG_LIBRARY_VERSION;
        msg = "Incompatible library version";
        break;
    default:
        errID = UNKNOWN_ERROR;
        msg = "Unknown error (" + std::to_string(rc) + ")";
        break;
    }

    if (stream.msg) {
        msg += ": " + std::string(stream.msg);
    }

    mexErrMsgIdAndTxt(errID, msg.c_str());
}


struct DeflateEnd {
    DeflateEnd(z_stream &stream) : stream(stream) { }
    ~DeflateEnd() { (void)deflateEnd(&stream); }
private:
    z_stream &stream;
};


mxArray * compress(int numInputs, const mxArray *inputs[]) {
    int windowBits = 15;
    int level = Z_DEFAULT_COMPRESSION;
    std::size_t inputDataSize = 0;
    z_stream stream{};  // Zero-initialized

    switch (numInputs) {
    case 4:
        windowBits = getInteger(inputs, 3);
    case 3:
        level = getInteger(inputs, 2);
    case 2:
        inputDataSize = getInputData(inputs, 1, stream);
        break;
    default:
        mxAssert(numInputs > 4, "Not enough inputs for compress");
        mexErrMsgIdAndTxt(WRONG_NUM_INPUTS,
                          "Too many inputs for '%s'",
                          COMPRESS_ACTION);
        break;
    }

    auto rc = deflateInit2(&stream,
                           level,
                           Z_DEFLATED,
                           windowBits,
                           8,
                           Z_DEFAULT_STRATEGY);
    if (Z_OK != rc) {
        throwZLibError(rc, stream);
    }
    DeflateEnd de(stream);

    auto outputDataSize = deflateBound(&stream, inputDataSize);
    auto outputData = createOutputData(outputDataSize, stream);

    rc = deflate(&stream, Z_FINISH);
    if (Z_STREAM_END != rc) {
        throwZLibError(rc, stream);
    }

    return createOutput(outputData, outputDataSize, stream);
}


struct InflateEnd {
    InflateEnd(z_stream &stream) : stream(stream) { }
    ~InflateEnd() { (void)inflateEnd(&stream); }
private:
    z_stream &stream;
};


mxArray * decompress(int numInputs, const mxArray *inputs[]) {
    int windowBits = 15;
    std::size_t inputDataSize = 0;
    z_stream stream{};  // Zero-initialized

    switch (numInputs) {
    case 3:
        windowBits = getInteger(inputs, 2);
    case 2:
        inputDataSize = getInputData(inputs, 1, stream);
        break;
    default:
        mxAssert(numInputs > 3, "Not enough inputs for decompress");
        mexErrMsgIdAndTxt(WRONG_NUM_INPUTS,
                          "Too many inputs for '%s'",
                          DECOMPRESS_ACTION);
        break;
    }

    auto rc = inflateInit2(&stream, windowBits);
    if (Z_OK != rc) {
        throwZLibError(rc, stream);
    }
    InflateEnd ie(stream);

    auto outputDataSize = inputDataSize * 2;
    auto outputData = createOutputData(outputDataSize, stream);

    do {
        rc = inflate(&stream, Z_SYNC_FLUSH);
        if (Z_OK != rc) {
            break;
        }
        // Output data is too small, so double its size
        auto newOutputDataSize = outputDataSize * 2;
        outputData = static_cast<std::uint8_t *>(mxRealloc(outputData,
                                                           newOutputDataSize));
        stream.next_out = outputData + outputDataSize;
        stream.avail_out = outputDataSize;
        outputDataSize = newOutputDataSize;
    } while (true);

    if (Z_STREAM_END != rc) {
        throwZLibError(rc, stream);
    }

    return createOutput(outputData, outputDataSize, stream);
}


ZLIB_MATLAB_NAMESPACE_END


void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    if (numOutputs != 1) {
        mexErrMsgIdAndTxt(WRONG_NUM_OUTPUTS, "One output required");
    }
    if (numInputs < 2) {
        mexErrMsgIdAndTxt(WRONG_NUM_INPUTS, "At least two inputs required");
    }

    auto action = getString(inputs, 0);
    mxArray *output = nullptr;

    if (action == COMPRESS_ACTION) {
        output = compress(numInputs, inputs);
    } else if (action == DECOMPRESS_ACTION) {
        output = decompress(numInputs, inputs);
    } else {
        mexErrMsgIdAndTxt(INVALID_INPUT,
                          "Unknown action: '%s'",
                          action.c_str());
    }

    outputs[0] = output;
}
