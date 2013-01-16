#ifndef RUNTIME_ERROR_H
#define RUNTIME_ERROR_H

#include <exception>
#include <string>

/**
 * \brief a run time error exception
 */
class RuntimeError : public std::exception
{
  public:
    /**
     * constructor which allows to give a error message
     * with printf like syntax
     */
    explicit RuntimeError(const char* fmt, ...)  __attribute__ ((format (printf, 2, 3)));
    virtual ~RuntimeError() throw();
    virtual const char* what() const throw() {return _errorMsg.c_str();}

  protected:
    std::string _errorMsg;
};

#endif
