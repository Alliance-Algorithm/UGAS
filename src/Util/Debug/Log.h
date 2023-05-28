#pragma once
/*
Creation Date: 2022/10/17
Latest Update: 2022/10/18
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 引用Easylogging++(https://github.com/amrayn/easyloggingpp)头文件
- 提供throw_with_trace宏在抛出错误时保存代码位置信息
- 抛出的异常必须继承自std:exception
- 每次抛出异常时将同步记录日志(Log.Error)，因此不应使用异常来控制程序流
*/

#include <exception>

#include "ThirdParty/EasyLogging++/EasyLogging++.h"


//定义宏__LINE_STR__，将被展开为宏所在行号的字符串形式
#define __INNER_INT_TO_STR_1(R) #R
#define __INNER_INT_TO_STR_2(R) __INNER_INT_TO_STR_1(R)
#define __LINE_STR__ __INNER_INT_TO_STR_2(__LINE__)


//定义跨平台宏__UNIVERSAL_FUNC__，将被展开为宏所在函数名
#ifdef __PRETTY_FUNCTION__
#define __UNIVERSAL_FUNC__ __PRETTY_FUNCTION__
#else
#ifdef __FUNCSIG__
#define __UNIVERSAL_FUNC__ __FUNCSIG__
#else
#ifdef __FUNCTION__
#define __UNIVERSAL_FUNC__ __FUNCTION__
#else
#ifdef __FUNC__
#define __UNIVERSAL_FUNC__ __FUNC__
#else
#ifdef __func__
#define __UNIVERSAL_FUNC__ __func__
#else
#define __UNIVERSAL_FUNC__ "unknown function"
#endif
#endif
#endif
#endif
#endif


// 为了和throw风格一致，不采用全大写
// 使用格式：throw std::xx_exception("错误内容"); ==> throw_with_trace(std:xx_exception, "错误内容");
// 所有抛出的错误类型必须继承自std::exception，错误内容必须为字符串
#define throw_with_trace(exception_name, message) \
{ \
    exception_name exception_to_throw = exception_name(message); \
    LOG(ERROR) << "<" << typeid(exception_to_throw).name() << ("> thrown in file \"" __FILE__ "\", line " __LINE_STR__ ", at [" __UNIVERSAL_FUNC__ "]"); \
    throw exception_to_throw; \
}


// 为了和throw风格一致，不采用全大写
// 使用格式：
// catch (std::xx_exception e) {
//     log_trace(e);
//     你的代码
//     throw;
// } 若要重新抛出错误，必须使用throw;而非throw(e);，后者会导致异常类型丢失。
#define log_trace(e) LOG(ERROR) << "<" << typeid(e).name() << ("> tracked in file \"" __FILE__ "\", line " __LINE_STR__ ", at [" __UNIVERSAL_FUNC__ "]");


// 为了和throw风格一致，不采用全大写
// 使用格式：
// catch (...) {
//     log_trace_unknown();
//     你的代码
//     throw;
// } 若要重新抛出错误，必须使用throw;而非throw(e);，后者会导致异常类型丢失。
#define log_trace_unknown() LOG(ERROR) << ("<UNKNOWN EXCEPTION> tracked in file \"" __FILE__ "\", line " __LINE_STR__ ", at [" __UNIVERSAL_FUNC__ "]");
