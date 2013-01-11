/****************************************************************************
** Meta object code from reading C++ file 'viewerGUI.h'
**
** Created: Tue Jan 8 00:22:44 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "viewerGUI.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'viewerGUI.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ViewerGUI[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      15,   11,   10,   10, 0x0a,
      31,   11,   10,   10, 0x0a,
      47,   11,   10,   10, 0x0a,
      63,   10,   10,   10, 0x0a,
      78,   10,   10,   10, 0x0a,
      95,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ViewerGUI[] = {
    "ViewerGUI\0\0val\0updateVal1(int)\0"
    "updateVal2(int)\0updateVal3(int)\0"
    "showOriginal()\0lineExtraction()\0"
    "setAlgorithm()\0"
};

void ViewerGUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ViewerGUI *_t = static_cast<ViewerGUI *>(_o);
        switch (_id) {
        case 0: _t->updateVal1((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->updateVal2((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->updateVal3((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->showOriginal(); break;
        case 4: _t->lineExtraction(); break;
        case 5: _t->setAlgorithm(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ViewerGUI::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ViewerGUI::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_ViewerGUI,
      qt_meta_data_ViewerGUI, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ViewerGUI::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ViewerGUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ViewerGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ViewerGUI))
        return static_cast<void*>(const_cast< ViewerGUI*>(this));
    if (!strcmp(_clname, "Ui::MainWindow"))
        return static_cast< Ui::MainWindow*>(const_cast< ViewerGUI*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int ViewerGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
