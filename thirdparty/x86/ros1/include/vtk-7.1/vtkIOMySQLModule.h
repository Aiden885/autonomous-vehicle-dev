
#ifndef VTKIOMYSQL_EXPORT_H
#define VTKIOMYSQL_EXPORT_H

#ifdef VTKIOMYSQL_STATIC_DEFINE
#  define VTKIOMYSQL_EXPORT
#  define VTKIOMYSQL_NO_EXPORT
#else
#  ifndef VTKIOMYSQL_EXPORT
#    ifdef vtkIOMySQL_EXPORTS
        /* We are building this library */
#      define VTKIOMYSQL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKIOMYSQL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKIOMYSQL_NO_EXPORT
#    define VTKIOMYSQL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKIOMYSQL_DEPRECATED
#  define VTKIOMYSQL_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKIOMYSQL_DEPRECATED_EXPORT VTKIOMYSQL_EXPORT __attribute__ ((__deprecated__))
#  define VTKIOMYSQL_DEPRECATED_NO_EXPORT VTKIOMYSQL_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOMYSQL_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkIOSQLModule.h"
#include "vtkIOSQLModule.h"

/* AutoInit implementations.  */
#if defined(vtkIOMySQL_INCLUDE)
# include vtkIOMySQL_INCLUDE
#endif
#if defined(vtkIOMySQL_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkIOMySQL)
#endif

#endif
