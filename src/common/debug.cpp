#include "debug.h"

#include <cassert>
#include <stdio.h>
#include <cstdarg>

///@file common/debug.cpp Debugging facilities @ingroup common

void message(const char* msg) {
	fprintf(stdout, "%s\n", msg);
    fflush(stderr);
}

void message_va(const char* msg, ...) {
    va_list args;
    va_start (args, msg);
    vfprintf(stdout, msg, args);
    va_end (args);
    fprintf(stdout, "\n");
    fflush(stdout);
}

void warning(const char* msg) {
    fprintf(stderr, "warning: %s\n", msg);
    fflush(stderr);
}

void warning_va(const char* msg, ...) {
    fprintf(stderr, "warning: ");
    va_list args;
    va_start (args, msg);
	vfprintf(stderr, msg, args);
    va_end (args);
    fprintf(stderr, "\n");
    fflush(stderr);
}

bool warning_if_not(bool check, const char* msg) {
    if(check) return check;
    fprintf(stderr, "warning: %s\n", msg);
    fflush(stderr);
    return check;
}

bool warning_if_not_va(bool check, const char* msg, ...) {
	if(check) return check;
    fprintf(stderr, "warning: ");
    va_list args;
    va_start (args, msg);
	vfprintf(stderr, msg, args);
    va_end (args);
    fprintf(stderr, "\n");
    fflush(stderr);
    return check;
}

void error(const char* msg) {
    fprintf(stderr, "error: %s\n", msg);
    fflush(stderr);
    assert(false);
}

void error_va(const char* msg, ...) {
    fprintf(stderr, "error: ");
    va_list args;
    va_start (args, msg);
	vfprintf(stderr, msg, args);
    va_end (args);
    fprintf(stderr, "\n");
    fflush(stderr);
	assert(false);
}

bool error_if_not(bool check, const char* msg) {
    if(check) return check;
    fprintf(stderr, "error: %s\n", msg);
    fflush(stderr);
    assert(check);
    return check;
}

bool error_if_not_va(bool check, const char* msg, ...) {
	if(check) return check;
    fprintf(stderr, "error: ");
    va_list args;
    va_start (args, msg);
	vfprintf(stderr, msg, args);
    va_end (args);
    fprintf(stderr, "\n");
    fflush(stderr);
	assert(check);
    return check;
}

void debug_break() {
    message("break: press any key to continue.");
    getchar();
}

void not_implemented_warning() {
    warning("not implemented");
}

