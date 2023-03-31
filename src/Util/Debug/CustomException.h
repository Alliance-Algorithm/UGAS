#pragma once
/*
Creation Date: 2022/10/19
Latest Update: 2022/10/19
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
*/

#include <string>
#include <stdexcept>

namespace customex {
    class open_video_error : public std::runtime_error {
    public:
        using _Mybase = std::runtime_error;

        explicit open_video_error(const std::string& _Message) : _Mybase(_Message.c_str()) {}

        explicit open_video_error(const char* _Message) : _Mybase(_Message) {}
    };
}
