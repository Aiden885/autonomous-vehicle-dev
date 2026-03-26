
#ifndef VTKIOFFMPEG_EXPORT_H
#define VTKIOFFMPEG_EXPORT_H

#ifdef VTKIOFFMPEG_STATIC_DEFINE
#  define VTKIOFFMPEG_EXPORT
#  define VTKIOFFMPEG_NO_EXPORT
#else
#  ifndef VTKIOFFMPEG_EXPORT
#    ifdef vtkIOFFMPEG_EXPORTS
        /* We are building this library */
#      define VTKIOFFMPEG_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKIOFFMPEG_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKIOFFMPEG_NO_EXPORT
#    define VTKIOFFMPEG_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKIOFFMPEG_DEPRECATED
#  define VTKIOFFMPEG_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKIOFFMPEG_DEPRECATED_EXPORT VTKIOFFMPEG_EXPORT __attribute__ ((__deprecated__))
#  define VTKIOFFMPEG_DEPRECATED_NO_EXPORT VTKIOFFMPEG_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOFFMPEG_NO_DEPRECATED
#endif



#endif
