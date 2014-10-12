
#ifdef EXCADE_API
#undef EXCADE_API
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# ifdef EXCADE_OPENHRP_MAKE_DLL
#  define EXCADE_API __declspec(dllexport)
# else
#  define EXCADE_API __declspec(dllimport)
# endif
#else
# define EXCADE_API
#endif
