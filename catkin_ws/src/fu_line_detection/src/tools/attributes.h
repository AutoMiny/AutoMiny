/**
 ** Definition of macros for attributes.
 */

#ifndef ATTRIBUTES_H__
#define ATTRIBUTES_H__

/** DEPRECATE
 **
 ** Define a function (or possibly class, depending on compiler) as deprecated.
 **
 ** Usage:
 **   DEPRECATED bool checkThis() { return true; }
 **
 ** Based on http://stackoverflow.com/questions/295120/c-mark-as-deprecated
 */
#ifdef __GNUC__
#define DEPRECATED __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif


#endif
