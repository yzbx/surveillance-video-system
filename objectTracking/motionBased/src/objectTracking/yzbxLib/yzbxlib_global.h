#ifndef YZBXLIB_GLOBAL_H
#define YZBXLIB_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(YZBXLIB_LIBRARY)
#  define YZBXLIBSHARED_EXPORT Q_DECL_EXPORT
#else
#  define YZBXLIBSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // YZBXLIB_GLOBAL_H
