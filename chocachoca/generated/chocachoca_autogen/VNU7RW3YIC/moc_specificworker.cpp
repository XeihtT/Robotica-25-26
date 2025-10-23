/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/specificworker.h"
#include <QtGui/qtextcursor.h>
#include <QScreen>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_SpecificWorker_t {
    uint offsetsAndSizes[26];
    char stringdata0[15];
    char stringdata1[16];
    char stringdata2[1];
    char stringdata3[11];
    char stringdata4[25];
    char stringdata5[12];
    char stringdata6[16];
    char stringdata7[6];
    char stringdata8[11];
    char stringdata9[8];
    char stringdata10[10];
    char stringdata11[8];
    char stringdata12[14];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_SpecificWorker_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_SpecificWorker_t qt_meta_stringdata_SpecificWorker = {
    {
        QT_MOC_LITERAL(0, 14),  // "SpecificWorker"
        QT_MOC_LITERAL(15, 15),  // "new_target_slot"
        QT_MOC_LITERAL(31, 0),  // ""
        QT_MOC_LITERAL(32, 10),  // "draw_lidar"
        QT_MOC_LITERAL(43, 24),  // "RoboCompLidar3D::TPoints"
        QT_MOC_LITERAL(68, 11),  // "filter_data"
        QT_MOC_LITERAL(80, 15),  // "QGraphicsScene*"
        QT_MOC_LITERAL(96, 5),  // "scene"
        QT_MOC_LITERAL(102, 10),  // "initialize"
        QT_MOC_LITERAL(113, 7),  // "compute"
        QT_MOC_LITERAL(121, 9),  // "emergency"
        QT_MOC_LITERAL(131, 7),  // "restore"
        QT_MOC_LITERAL(139, 13)   // "startup_check"
    },
    "SpecificWorker",
    "new_target_slot",
    "",
    "draw_lidar",
    "RoboCompLidar3D::TPoints",
    "filter_data",
    "QGraphicsScene*",
    "scene",
    "initialize",
    "compute",
    "emergency",
    "restore",
    "startup_check"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_SpecificWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   56,    2, 0x0a,    1 /* Public */,
       3,    2,   59,    2, 0x0a,    3 /* Public */,
       8,    0,   64,    2, 0x0a,    6 /* Public */,
       9,    0,   65,    2, 0x0a,    7 /* Public */,
      10,    0,   66,    2, 0x0a,    8 /* Public */,
      11,    0,   67,    2, 0x0a,    9 /* Public */,
      12,    0,   68,    2, 0x0a,   10 /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, 0x80000000 | 4, 0x80000000 | 6,    5,    7,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Int,

       0        // eod
};

Q_CONSTINIT const QMetaObject SpecificWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<GenericWorker::staticMetaObject>(),
    qt_meta_stringdata_SpecificWorker.offsetsAndSizes,
    qt_meta_data_SpecificWorker,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_SpecificWorker_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<SpecificWorker, std::true_type>,
        // method 'new_target_slot'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QPointF, std::false_type>,
        // method 'draw_lidar'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const RoboCompLidar3D::TPoints &, std::false_type>,
        QtPrivate::TypeAndForceComplete<QGraphicsScene *, std::false_type>,
        // method 'initialize'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'compute'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'emergency'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'restore'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'startup_check'
        QtPrivate::TypeAndForceComplete<int, std::false_type>
    >,
    nullptr
} };

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SpecificWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->new_target_slot((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1]))); break;
        case 1: _t->draw_lidar((*reinterpret_cast< std::add_pointer_t<RoboCompLidar3D::TPoints>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QGraphicsScene*>>(_a[2]))); break;
        case 2: _t->initialize(); break;
        case 3: _t->compute(); break;
        case 4: _t->emergency(); break;
        case 5: _t->restore(); break;
        case 6: { int _r = _t->startup_check();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 1:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QGraphicsScene* >(); break;
            }
            break;
        }
    }
}

const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker.stringdata0))
        return static_cast<void*>(this);
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
