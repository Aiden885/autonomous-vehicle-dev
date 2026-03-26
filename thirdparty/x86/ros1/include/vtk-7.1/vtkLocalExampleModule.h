
#ifndef VTKLOCALEXAMPLE_EXPORT_H
#define VTKLOCALEXAMPLE_EXPORT_H

#ifdef VTKLOCALEXAMPLE_STATIC_DEFINE
#  define VTKLOCALEXAMPLE_EXPORT
#  define VTKLOCALEXAMPLE_NO_EXPORT
#else
#  ifndef VTKLOCALEXAMPLE_EXPORT
#    ifdef vtkLocalExample_EXPORTS
        /* We are building this library */
#      define VTKLOCALEXAMPLE_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKLOCALEXAMPLE_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKLOCALEXAMPLE_NO_EXPORT
#    define VTKLOCALEXAMPLE_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKLOCALEXAMPLE_DEPRECATED
#  define VTKLOCALEXAMPLE_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKLOCALEXAMPLE_DEPRECATED_EXPORT VTKLOCALEXAMPLE_EXPORT __attribute__ ((__deprecated__))
#  define VTKLOCALEXAMPLE_DEPRECATED_NO_EXPORT VTKLOCALEXAMPLE_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKLOCALEXAMPLE_NO_DEPRECATED
#endif



#endif
