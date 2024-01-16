/****************************************************************************
** Meta object code from reading C++ file 'TypeWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.12)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../TypeWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'TypeWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.12. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_TypeWidget_t {
    QByteArrayData data[10];
    char stringdata0[165];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TypeWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TypeWidget_t qt_meta_stringdata_TypeWidget = {
    {
QT_MOC_LITERAL(0, 0, 10), // "TypeWidget"
QT_MOC_LITERAL(1, 11, 20), // "sendNewTypeToMainWin"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 4), // "data"
QT_MOC_LITERAL(4, 38, 21), // "sendEditTypeToMainWin"
QT_MOC_LITERAL(5, 60, 26), // "receiveEditTypeFromMainWin"
QT_MOC_LITERAL(6, 87, 18), // "onCoatOkBtnClicked"
QT_MOC_LITERAL(7, 106, 20), // "onCoatSaveBtnClicked"
QT_MOC_LITERAL(8, 127, 17), // "onDieOkBtnClicked"
QT_MOC_LITERAL(9, 145, 19) // "onDieSaveBtnClicked"

    },
    "TypeWidget\0sendNewTypeToMainWin\0\0data\0"
    "sendEditTypeToMainWin\0receiveEditTypeFromMainWin\0"
    "onCoatOkBtnClicked\0onCoatSaveBtnClicked\0"
    "onDieOkBtnClicked\0onDieSaveBtnClicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TypeWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       4,    1,   52,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   55,    2, 0x0a /* Public */,
       6,    0,   58,    2, 0x0a /* Public */,
       7,    0,   59,    2, 0x0a /* Public */,
       8,    0,   60,    2, 0x0a /* Public */,
       9,    0,   61,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QStringList,    3,
    QMetaType::Void, QMetaType::QStringList,    3,

 // slots: parameters
    QMetaType::Void, QMetaType::QStringList,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void TypeWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TypeWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendNewTypeToMainWin((*reinterpret_cast< QStringList(*)>(_a[1]))); break;
        case 1: _t->sendEditTypeToMainWin((*reinterpret_cast< QStringList(*)>(_a[1]))); break;
        case 2: _t->receiveEditTypeFromMainWin((*reinterpret_cast< QStringList(*)>(_a[1]))); break;
        case 3: _t->onCoatOkBtnClicked(); break;
        case 4: _t->onCoatSaveBtnClicked(); break;
        case 5: _t->onDieOkBtnClicked(); break;
        case 6: _t->onDieSaveBtnClicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (TypeWidget::*)(QStringList );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&TypeWidget::sendNewTypeToMainWin)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (TypeWidget::*)(QStringList );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&TypeWidget::sendEditTypeToMainWin)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject TypeWidget::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_TypeWidget.data,
    qt_meta_data_TypeWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *TypeWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TypeWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_TypeWidget.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int TypeWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void TypeWidget::sendNewTypeToMainWin(QStringList _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void TypeWidget::sendEditTypeToMainWin(QStringList _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
