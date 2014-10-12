//
// C++ Interface: exception
//
// Description:
//
//
// Author: Jonas Haller
//
// Copyright: Neuronics AG, 2008
//
//

#ifndef _EXCEPTION_H_
#define _EXCEPTION_H_

#include <string>

namespace AnaGuess {

///
/// @defgroup exceptions Exceptions
/// @{
///

class Exception : public std::exception {
    protected:
        const std::string mMessage;
        const int mErrorNumber;

    public:
        Exception(const std::string & aMessage, const int aErrorNumber) throw () :
            std::exception(),
            mMessage(aMessage), mErrorNumber(aErrorNumber) {
        }

        virtual ~Exception() throw () {
        }

        std::string message() const throw() {
            return mMessage;
        }
	const char* what() const throw() {
	    return mMessage.c_str();
	}

	const int errorNumber() const throw() {
	    return mErrorNumber;
	}
};

///
/// @}
///

} // namespace

#endif //_EXCEPTION_H_
