/********************************************************************************
** Form generated from reading UI file 'dm_base_main_window.ui'
**
** Created: Wed Dec 19 11:07:07 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DM_BASE_MAIN_WINDOW_H
#define UI_DM_BASE_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "dm_qglviewer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionQuit;
    QAction *actionPoints;
    QAction *actionNormals;
    QAction *actionCovariances;
    QAction *actionCorrespndences;
    QAction *actionStep;
    QAction *actionAbout;
    QAction *actionMinimize;
    QAction *actionMaximize;
    QWidget *centralwidget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox_drawing_options_3d;
    QCheckBox *checkBox_points;
    QCheckBox *checkBox_covariances;
    QCheckBox *checkBox_correspondences;
    QCheckBox *checkBox_normals;
    QCheckBox *checkBox_step;
    QSpinBox *spinBox_step;
    QDoubleSpinBox *doubleSpinBox_normals;
    QDoubleSpinBox *doubleSpinBox_covariances;
    QDoubleSpinBox *doubleSpinBox_points;
    QDoubleSpinBox *doubleSpinBox_correspondences;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QGroupBox *groupBox_viewer_3d;
    DMQGLViewer *viewer_3d;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBox_viewer_2d;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuHelp;
    QMenu *menuWindow;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionPoints = new QAction(MainWindow);
        actionPoints->setObjectName(QString::fromUtf8("actionPoints"));
        actionNormals = new QAction(MainWindow);
        actionNormals->setObjectName(QString::fromUtf8("actionNormals"));
        actionCovariances = new QAction(MainWindow);
        actionCovariances->setObjectName(QString::fromUtf8("actionCovariances"));
        actionCorrespndences = new QAction(MainWindow);
        actionCorrespndences->setObjectName(QString::fromUtf8("actionCorrespndences"));
        actionStep = new QAction(MainWindow);
        actionStep->setObjectName(QString::fromUtf8("actionStep"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionMinimize = new QAction(MainWindow);
        actionMinimize->setObjectName(QString::fromUtf8("actionMinimize"));
        actionMaximize = new QAction(MainWindow);
        actionMaximize->setObjectName(QString::fromUtf8("actionMaximize"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 10, 241, 191));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        groupBox_drawing_options_3d = new QGroupBox(verticalLayoutWidget);
        groupBox_drawing_options_3d->setObjectName(QString::fromUtf8("groupBox_drawing_options_3d"));
        groupBox_drawing_options_3d->setEnabled(true);
        checkBox_points = new QCheckBox(groupBox_drawing_options_3d);
        checkBox_points->setObjectName(QString::fromUtf8("checkBox_points"));
        checkBox_points->setGeometry(QRect(13, 60, 71, 20));
        checkBox_covariances = new QCheckBox(groupBox_drawing_options_3d);
        checkBox_covariances->setObjectName(QString::fromUtf8("checkBox_covariances"));
        checkBox_covariances->setGeometry(QRect(13, 120, 111, 20));
        checkBox_correspondences = new QCheckBox(groupBox_drawing_options_3d);
        checkBox_correspondences->setObjectName(QString::fromUtf8("checkBox_correspondences"));
        checkBox_correspondences->setGeometry(QRect(13, 150, 151, 20));
        checkBox_normals = new QCheckBox(groupBox_drawing_options_3d);
        checkBox_normals->setObjectName(QString::fromUtf8("checkBox_normals"));
        checkBox_normals->setGeometry(QRect(13, 90, 91, 20));
        checkBox_step = new QCheckBox(groupBox_drawing_options_3d);
        checkBox_step->setObjectName(QString::fromUtf8("checkBox_step"));
        checkBox_step->setGeometry(QRect(13, 29, 61, 22));
        spinBox_step = new QSpinBox(groupBox_drawing_options_3d);
        spinBox_step->setObjectName(QString::fromUtf8("spinBox_step"));
        spinBox_step->setEnabled(false);
        spinBox_step->setGeometry(QRect(170, 27, 62, 27));
        spinBox_step->setMinimum(1);
        doubleSpinBox_normals = new QDoubleSpinBox(groupBox_drawing_options_3d);
        doubleSpinBox_normals->setObjectName(QString::fromUtf8("doubleSpinBox_normals"));
        doubleSpinBox_normals->setEnabled(false);
        doubleSpinBox_normals->setGeometry(QRect(170, 87, 62, 27));
        doubleSpinBox_normals->setSingleStep(0.01);
        doubleSpinBox_covariances = new QDoubleSpinBox(groupBox_drawing_options_3d);
        doubleSpinBox_covariances->setObjectName(QString::fromUtf8("doubleSpinBox_covariances"));
        doubleSpinBox_covariances->setEnabled(false);
        doubleSpinBox_covariances->setGeometry(QRect(170, 117, 62, 27));
        doubleSpinBox_covariances->setSingleStep(0.01);
        doubleSpinBox_points = new QDoubleSpinBox(groupBox_drawing_options_3d);
        doubleSpinBox_points->setObjectName(QString::fromUtf8("doubleSpinBox_points"));
        doubleSpinBox_points->setEnabled(false);
        doubleSpinBox_points->setGeometry(QRect(170, 57, 62, 27));
        doubleSpinBox_points->setSingleStep(0.25);
        doubleSpinBox_correspondences = new QDoubleSpinBox(groupBox_drawing_options_3d);
        doubleSpinBox_correspondences->setObjectName(QString::fromUtf8("doubleSpinBox_correspondences"));
        doubleSpinBox_correspondences->setEnabled(false);
        doubleSpinBox_correspondences->setGeometry(QRect(170, 147, 62, 27));
        doubleSpinBox_correspondences->setSingleStep(0.25);

        verticalLayout->addWidget(groupBox_drawing_options_3d);

        horizontalLayoutWidget = new QWidget(centralwidget);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(260, 10, 531, 531));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        groupBox_viewer_3d = new QGroupBox(horizontalLayoutWidget);
        groupBox_viewer_3d->setObjectName(QString::fromUtf8("groupBox_viewer_3d"));
        viewer_3d = new DMQGLViewer(groupBox_viewer_3d);
        viewer_3d->setObjectName(QString::fromUtf8("viewer_3d"));
        viewer_3d->setGeometry(QRect(9, 29, 511, 491));

        horizontalLayout->addWidget(groupBox_viewer_3d);

        horizontalLayoutWidget_2 = new QWidget(centralwidget);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(10, 210, 241, 331));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        groupBox_viewer_2d = new QGroupBox(horizontalLayoutWidget_2);
        groupBox_viewer_2d->setObjectName(QString::fromUtf8("groupBox_viewer_2d"));
        groupBox_viewer_2d->setEnabled(true);

        horizontalLayout_2->addWidget(groupBox_viewer_2d);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 25));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menubar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        menuWindow = new QMenu(menubar);
        menuWindow->setObjectName(QString::fromUtf8("menuWindow"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuView->menuAction());
        menubar->addAction(menuWindow->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuView->addSeparator();

        retranslateUi(MainWindow);
        QObject::connect(checkBox_step, SIGNAL(toggled(bool)), spinBox_step, SLOT(setEnabled(bool)));
        QObject::connect(checkBox_points, SIGNAL(toggled(bool)), doubleSpinBox_points, SLOT(setEnabled(bool)));
        QObject::connect(checkBox_normals, SIGNAL(toggled(bool)), doubleSpinBox_normals, SLOT(setEnabled(bool)));
        QObject::connect(checkBox_covariances, SIGNAL(toggled(bool)), doubleSpinBox_covariances, SLOT(setEnabled(bool)));
        QObject::connect(checkBox_correspondences, SIGNAL(toggled(bool)), doubleSpinBox_correspondences, SLOT(setEnabled(bool)));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionQuit->setText(QApplication::translate("MainWindow", "Quit", 0, QApplication::UnicodeUTF8));
        actionPoints->setText(QApplication::translate("MainWindow", "Points", 0, QApplication::UnicodeUTF8));
        actionNormals->setText(QApplication::translate("MainWindow", "Normals", 0, QApplication::UnicodeUTF8));
        actionCovariances->setText(QApplication::translate("MainWindow", "Covariances", 0, QApplication::UnicodeUTF8));
        actionCorrespndences->setText(QApplication::translate("MainWindow", "Correspndences", 0, QApplication::UnicodeUTF8));
        actionStep->setText(QApplication::translate("MainWindow", "Step", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindow", "About DM Viewer", 0, QApplication::UnicodeUTF8));
        actionMinimize->setText(QApplication::translate("MainWindow", "Minimize", 0, QApplication::UnicodeUTF8));
        actionMaximize->setText(QApplication::translate("MainWindow", "Maximize", 0, QApplication::UnicodeUTF8));
        groupBox_drawing_options_3d->setTitle(QApplication::translate("MainWindow", "3D Viewer Drawing Options", 0, QApplication::UnicodeUTF8));
        checkBox_points->setText(QApplication::translate("MainWindow", "Points", 0, QApplication::UnicodeUTF8));
        checkBox_covariances->setText(QApplication::translate("MainWindow", "Covariances", 0, QApplication::UnicodeUTF8));
        checkBox_correspondences->setText(QApplication::translate("MainWindow", "Correspondences", 0, QApplication::UnicodeUTF8));
        checkBox_normals->setText(QApplication::translate("MainWindow", "Normals", 0, QApplication::UnicodeUTF8));
        checkBox_step->setText(QApplication::translate("MainWindow", "Step", 0, QApplication::UnicodeUTF8));
        groupBox_viewer_3d->setTitle(QApplication::translate("MainWindow", "3D Viewer", 0, QApplication::UnicodeUTF8));
        groupBox_viewer_2d->setTitle(QApplication::translate("MainWindow", "2D Viewers", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuView->setTitle(QApplication::translate("MainWindow", "View", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
        menuWindow->setTitle(QApplication::translate("MainWindow", "Window", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DM_BASE_MAIN_WINDOW_H
