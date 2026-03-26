
#ifndef VTKWRAPPINGJAVA_EXPORT_H
#define VTKWRAPPINGJAVA_EXPORT_H

#ifdef VTKWRAPPINGJAVA_STATIC_DEFINE
#  define VTKWRAPPINGJAVA_EXPORT
#  define VTKWRAPPINGJAVA_NO_EXPORT
#else
#  ifndef VTKWRAPPINGJAVA_EXPORT
#    ifdef vtkWrappingJava_EXPORTS
        /* We are building this library */
#      define VTKWRAPPINGJAVA_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKWRAPPINGJAVA_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKWRAPPINGJAVA_NO_EXPORT
#    define VTKWRAPPINGJAVA_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKWRAPPINGJAVA_DEPRECATED
#  define VTKWRAPPINGJAVA_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKWRAPPINGJAVA_DEPRECATED_EXPORT VTKWRAPPINGJAVA_EXPORT __attribute__ ((__deprecated__))
#  define VTKWRAPPINGJAVA_DEPRECATED_NO_EXPORT VTKWRAPPINGJAVA_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKWRAPPINGJAVA_NO_DEPRECATED
#endif



#endif
