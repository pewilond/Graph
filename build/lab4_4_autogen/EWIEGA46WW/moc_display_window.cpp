/****************************************************************************
** Meta object code from reading C++ file 'display_window.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../display_window.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'display_window.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DisplayWindow_t {
    QByteArrayData data[14];
    char stringdata0[174];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DisplayWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DisplayWindow_t qt_meta_stringdata_DisplayWindow = {
    {
QT_MOC_LITERAL(0, 0, 13), // "DisplayWindow"
QT_MOC_LITERAL(1, 14, 11), // "runAllTests"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 11), // "loadCsvData"
QT_MOC_LITERAL(4, 39, 21), // "buildPerformanceChart"
QT_MOC_LITERAL(5, 61, 9), // "addVertex"
QT_MOC_LITERAL(6, 71, 12), // "removeVertex"
QT_MOC_LITERAL(7, 84, 7), // "addEdge"
QT_MOC_LITERAL(8, 92, 10), // "removeEdge"
QT_MOC_LITERAL(9, 103, 14), // "removeEdgeById"
QT_MOC_LITERAL(10, 118, 9), // "randomAdd"
QT_MOC_LITERAL(11, 128, 11), // "runDijkstra"
QT_MOC_LITERAL(12, 140, 16), // "exportGraphToDot"
QT_MOC_LITERAL(13, 157, 16) // "renderGraphImage"

    },
    "DisplayWindow\0runAllTests\0\0loadCsvData\0"
    "buildPerformanceChart\0addVertex\0"
    "removeVertex\0addEdge\0removeEdge\0"
    "removeEdgeById\0randomAdd\0runDijkstra\0"
    "exportGraphToDot\0renderGraphImage"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DisplayWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x08 /* Private */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    0,   85,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void DisplayWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DisplayWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->runAllTests(); break;
        case 1: _t->loadCsvData(); break;
        case 2: _t->buildPerformanceChart(); break;
        case 3: _t->addVertex(); break;
        case 4: _t->removeVertex(); break;
        case 5: _t->addEdge(); break;
        case 6: _t->removeEdge(); break;
        case 7: _t->removeEdgeById(); break;
        case 8: _t->randomAdd(); break;
        case 9: _t->runDijkstra(); break;
        case 10: _t->exportGraphToDot(); break;
        case 11: _t->renderGraphImage(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject DisplayWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_DisplayWindow.data,
    qt_meta_data_DisplayWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DisplayWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DisplayWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DisplayWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int DisplayWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
