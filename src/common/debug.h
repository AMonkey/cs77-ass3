#ifndef DEBUG_H_
#define DEBUG_H_

///@file common/debug.h Debugging facilities @ingroup common
///@defgroup debug Debugging facilities
///@ingroup common
///@{

/// Prints a message
void message(const char* msg);
/// Prints a message (printf style)
void message_va(const char* msg, ...);

/// Prints a warning
void warning(const char* msg);
/// Prints a warning (printf style)
void warning_va(const char* msg, ...);

/// Prints a warning if a condition does not hold
bool warning_if_not(bool check, const char* msg);
/// Prints a warning if a condition does not hold (printf style)
bool warning_if_not_va(bool check, const char* msg, ...);

/// Prints an error message and stops
void error(const char* msg);
/// Prints an error message and stops (printf style)
void error_va(const char* msg, ...);

/// Prints an error message and stops if a condition does not hold
bool error_if_not(bool check, const char* msg);
/// Prints an error message and stops if a condition does not hold (printf style)
bool error_if_not_va(bool check, const char* msg, ...);

/// Break into the debugger
void debug_break();

/// Error signaling unimplemented features
#define not_implemented_error() \
    do { error_va("function %s not implemented at %s:%d.", __FUNCTION__, __FILE__, __LINE__); } while(0);

/// Error signaling unimplemented features with custom message
#define not_implemented_error_message(msg) \
    do { error_va("function %s not implemented at %s:%d.  %s", __FUNCTION__, __FILE__, __LINE__, msg); } while(0);

/// Error signaling unimplemented features
#define put_your_code_here(msg) \
    do { warning_va( "function %s is not (fully) implemented at %s:%d.  %s", __FUNCTION__, __FILE__, __LINE__, msg); } while(0);


///@}

#endif

