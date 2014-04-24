/********************************************************************************
** Form generated from reading UI file 'graph_gui.ui'
**
** Created: Tue Apr 22 17:54:47 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRAPH_GUI_H
#define UI_GRAPH_GUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "viewer.h"

QT_BEGIN_NAMESPACE

class Ui_GraphGUI
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_2;
    GraphViewer *graph_viewer;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QSpacerItem *verticalSpacer;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *GraphGUI)
    {
        if (GraphGUI->objectName().isEmpty())
            GraphGUI->setObjectName(QString::fromUtf8("GraphGUI"));
        GraphGUI->resize(953, 808);
        centralwidget = new QWidget(GraphGUI);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout_2 = new QHBoxLayout(centralwidget);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        graph_viewer = new GraphViewer(centralwidget);
        graph_viewer->setObjectName(QString::fromUtf8("graph_viewer"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(graph_viewer->sizePolicy().hasHeightForWidth());
        graph_viewer->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(graph_viewer);

        horizontalSpacer = new QSpacerItem(5, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(10);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetFixedSize);
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pushButton->sizePolicy().hasHeightForWidth());
        pushButton->setSizePolicy(sizePolicy1);
        pushButton->setMinimumSize(QSize(200, 0));

        verticalLayout->addWidget(pushButton);

        pushButton_2 = new QPushButton(centralwidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(pushButton_2->sizePolicy().hasHeightForWidth());
        pushButton_2->setSizePolicy(sizePolicy2);

        verticalLayout->addWidget(pushButton_2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);


        horizontalLayout_2->addLayout(horizontalLayout);

        GraphGUI->setCentralWidget(centralwidget);
        statusBar = new QStatusBar(GraphGUI);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        GraphGUI->setStatusBar(statusBar);

        retranslateUi(GraphGUI);

        QMetaObject::connectSlotsByName(GraphGUI);
    } // setupUi

    void retranslateUi(QMainWindow *GraphGUI)
    {
        GraphGUI->setWindowTitle(QApplication::translate("GraphGUI", "MainWindow", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("GraphGUI", "Full Merge", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("GraphGUI", "One Step Merge", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class GraphGUI: public Ui_GraphGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRAPH_GUI_H
