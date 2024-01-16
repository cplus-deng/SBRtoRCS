#ifndef _SILENCE_AMP_DEPRECATION_WARNINGS
#define _SILENCE_AMP_DEPRECATION_WARNINGS
#endif // !_SILENCE_AMP_DEPRECATION_WARNINGS
#include "SBRApp.h"
#include <QtWidgets/QApplication>
#include <QMetaType>
#include <vector>
typedef float Float;
typedef std::uint32_t U32;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType< std::vector<Float> >("std::vector<Float>");
    qRegisterMetaType<U32>("U32");
    SBRApp w;
    w.show();
    return a.exec();
}
