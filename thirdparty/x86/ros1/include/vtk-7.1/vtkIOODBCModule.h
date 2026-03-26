
#ifndef VTKIOODBC_EXPORT_H
#define VTKIOODBC_EXPORT_H

#ifdef VTKIOODBC_STATIC_DEFINE
#  define VTKIOODBC_EXPORT
#  define VTKIOODBC_NO_EXPORT
#else
#  ifndef VTKIOODBC_EXPORT
#    ifdef vtkIOODBC_EXPORTS
        /* We are building this library */
#      define VTKIOODBC_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKIOODBC_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKIOODBC_NO_EXPORT
#    define VTKIOODBC_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKIOODBC_DEPRECATED
#  define VTKIOODBC_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKIOODBC_DEPRECATED_EXPORT VTKIOODBC_EXPORT __attribute__ ((__deprecated__))
#  define VTKIOODBC_DEPRECATED_NO_EXPORT VTKIOODBC_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOODBC_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkIOSQLModule.h"

#endif
