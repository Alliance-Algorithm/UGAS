#pragma once
#include <stdexcept>

class BaseException : public std::exception {
public:
    BaseException(const char* str) :exception(str) {}
};

class BaseBoardException : public BaseException {
public:
    BaseBoardException(const char* str) :BaseException(str) {}
};

class BaseCaptureException : public BaseException {
public:
    BaseCaptureException(const char* str) :BaseException(str) {}
};

class CVVideoCaptureException : public  BaseCaptureException {
public:
    CVVideoCaptureException(const char* str) :BaseCaptureException(str) {}
};

class HikCameraCaptureException : public  BaseCaptureException {
public:
    HikCameraCaptureException(const char* str) :BaseCaptureException(str) {}
};

class HTCameraCaptureException : public  BaseCaptureException {
public:
    HTCameraCaptureException(const char* str) :BaseCaptureException(str) {}
};

class CBoardInfantryException : public  BaseBoardException {
public:
    CBoardInfantryException(const char* str) :BaseBoardException(str) {}
};
