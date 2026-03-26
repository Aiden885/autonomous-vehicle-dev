
#ifndef VTKIOPOSTGRESQL_EXPORT_H
#define VTKIOPOSTGRESQL_EXPORT_H

#ifdef VTKIOPOSTGRESQL_STATIC_DEFINE
#  define VTKIOPOSTGRESQL_EXPORT
#  define VTKIOPOSTGRESQL_NO_EXPORT
#else
#  ifndef VTKIOPOSTGRESQL_EXPORT
#    ifdef vtkIOPostgreSQL_EXPORTS
        /* We are building this library */
#      define VTKIOPOSTGRESQL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKIOPOSTGRESQL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKIOPOSTGRESQL_NO_EXPORT
#    define VTKIOPOSTGRESQL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKIOPOSTGRESQL_DEPRECATED
#  define VTKIOPOSTGRESQL_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKIOPOSTGRESQL_DEPRECATED_EXPORT VTKIOPOSTGRESQL_EXPORT __attribute__ ((__deprecated__))
#  define VTKIOPOSTGRESQL_DEPRECATED_NO_EXPORT VTKIOPOSTGRESQL_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOPOSTGRESQL_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkIOSQLModule.h"
#include "vtkIOSQLModule.h"

/* AutoInit implementations.  */
#if defined(vtkIOPostgreSQL_INCLUDE)
# include vtkIOPostgreSQL_INCLUDE
#endif
#if defined(vtkIOPostgreSQL_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkIOPostgreSQL)
#endif

#endif
