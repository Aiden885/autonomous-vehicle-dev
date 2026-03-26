
#ifndef VTKIOPARALLELNETCDF_EXPORT_H
#define VTKIOPARALLELNETCDF_EXPORT_H

#ifdef VTKIOPARALLELNETCDF_STATIC_DEFINE
#  define VTKIOPARALLELNETCDF_EXPORT
#  define VTKIOPARALLELNETCDF_NO_EXPORT
#else
#  ifndef VTKIOPARALLELNETCDF_EXPORT
#    ifdef vtkIOParallelNetCDF_EXPORTS
        /* We are building this library */
#      define VTKIOPARALLELNETCDF_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKIOPARALLELNETCDF_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKIOPARALLELNETCDF_NO_EXPORT
#    define VTKIOPARALLELNETCDF_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKIOPARALLELNETCDF_DEPRECATED
#  define VTKIOPARALLELNETCDF_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKIOPARALLELNETCDF_DEPRECATED_EXPORT VTKIOPARALLELNETCDF_EXPORT __attribute__ ((__deprecated__))
#  define VTKIOPARALLELNETCDF_DEPRECATED_NO_EXPORT VTKIOPARALLELNETCDF_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOPARALLELNETCDF_NO_DEPRECATED
#endif



#endif
