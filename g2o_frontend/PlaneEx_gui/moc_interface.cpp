/****************************************************************************
** Meta object code from reading C++ file 'interface.h'
**
** Created: Fri Jan 18 15:33:30 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "interface.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'interface.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ViewerInterface[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   17,   16,   16, 0x0a,
      37,   17,   16,   16, 0x0a,
      53,   17,   16,   16, 0x0a,
      66,   16,   16,   16, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ViewerInterface[] = {
    "ViewerInterface\0\0val\0updateZmin(int)\0"
    "updateZmax(int)\0updateP(int)\0showFull()\0"
};

void ViewerInterface::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ViewerInterface *_t = static_cast<ViewerInterface *>(_o);
        switch (_id) {
        case 0: _t->updateZmin((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->updateZmax((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->updateP((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->showFull(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ViewerInterface::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ViewerInterface::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_ViewerInterface,
      qt_meta_data_ViewerInterface, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ViewerInterface::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ViewerInterface::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ViewerInterface::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ViewerInterface))
        return static_cast<void*>(const_cast< ViewerInterface*>(this));
    if (!strcmp(_clname, "Ui::MainWindow"))
        return static_cast< Ui::MainWindow*>(const_cast< ViewerInterface*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int ViewerInterface::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
