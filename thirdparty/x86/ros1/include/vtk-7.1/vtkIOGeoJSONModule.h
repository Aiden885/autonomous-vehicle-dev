
#ifndef VTKIOGEOJSON_EXPORT_H
#define VTKIOGEOJSON_EXPORT_H

#ifdef VTKIOGEOJSON_STATIC_DEFINE
#  define VTKIOGEOJSON_EXPORT
#  define VTKIOGEOJSON_NO_EXPORT
#else
#  ifndef VTKIOGEOJSON_EXPORT
#    ifdef vtkIOGeoJSON_EXPORTS
        /* We are building this library */
#      define VTKIOGEOJSON_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKIOGEOJSON_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKIOGEOJSON_NO_EXPORT
#    define VTKIOGEOJSON_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKIOGEOJSON_DEPRECATED
#  define VTKIOGEOJSON_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKIOGEOJSON_DEPRECATED_EXPORT VTKIOGEOJSON_EXPORT __attribute__ ((__deprecated__))
#  define VTKIOGEOJSON_DEPRECATED_NO_EXPORT VTKIOGEOJSON_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOGEOJSON_NO_DEPRECATED
#endif



#endif
