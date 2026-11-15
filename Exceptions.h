// Exceptions.h - one exception type per failure domain, all derived from
// LogisticsError so callers can catch narrowly or broadly.
#pragma once
#include <stdexcept>

namespace logi {

class LogisticsError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class NetworkError : public LogisticsError {  // HTTP / libcurl failure
public:
    using LogisticsError::LogisticsError;
};
class ParseError : public LogisticsError {  // malformed JSON / file content
public:
    using LogisticsError::LogisticsError;
};
class FileError : public LogisticsError {  // cannot open / write a file
public:
    using LogisticsError::LogisticsError;
};
class RoutingError : public LogisticsError {  // no route / invalid plan
public:
    using LogisticsError::LogisticsError;
};

}  // namespace logi
