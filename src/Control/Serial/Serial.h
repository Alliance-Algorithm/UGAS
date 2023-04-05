#pragma once

#include "SerialDefines.h"
#include "Util/Debug/Log.h"

#ifdef _WIN32
#include "Impl/Windows.h"
#elif __linux__
#include "Impl/Linux.h"
#endif