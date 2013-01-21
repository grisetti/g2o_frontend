/********************************************************************************
 ** Form generated from reading UI file 'viewer_slider.ui'
 **
 ** Created: Tue Nov 27 00:05:38 2012
 **      by: Qt User Interface Compiler version 4.8.1
 **
 ** WARNING! All changes made in this file will be lost when recompiling UI file!
 ********************************************************************************/

#ifndef UI_VIEWER_SLIDER_H
#define UI_VIEWER_SLIDER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QSlider>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>
#include "QGLViewer/qglviewer.h"
#include "OverridedViewer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
	QWidget *centralwidget;
	OverridedViewer *viewer;
	QSlider *Slider1;
	QSlider *Slider2;
	QSlider *Slider3;
	QLabel *label;
	QLabel *label_2;
	QLabel *label_3;
	QPushButton *pushButton;
	QPushButton *pushButton_2;
	QMenuBar *menubar;
	QStatusBar *statusbar;

	void setupUi(QMainWindow *MainWindow)
	{
		if (MainWindow->objectName().isEmpty())
			MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
		MainWindow->resize(800, 600);
		centralwidget = new QWidget(MainWindow);
		centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
		viewer = new OverridedViewer(centralwidget);
		viewer->setObjectName(QString::fromUtf8("viewer"));
		viewer->setGeometry(QRect(0, 0, 621, 561));
		Slider1 = new QSlider(centralwidget);
		Slider1->setObjectName(QString::fromUtf8("Slider1"));
		Slider1->setGeometry(QRect(630, 40, 160, 20));
		Slider1->setOrientation(Qt::Horizontal);
		Slider1->setMinimum(0);
		Slider1->setMaximum(1000);
		Slider2 = new QSlider(centralwidget);
		Slider2->setObjectName(QString::fromUtf8("Slider2"));
		Slider2->setGeometry(QRect(630, 80, 160, 20));
		Slider2->setOrientation(Qt::Horizontal);
		Slider2->setMinimum(0);
		Slider2->setMaximum(10000);
		Slider3 = new QSlider(centralwidget);
		Slider3->setObjectName(QString::fromUtf8("Slider3"));
		Slider3->setGeometry(QRect(630, 122, 160, 20));
		Slider3->setOrientation(Qt::Horizontal);
		Slider3->setMinimum(0);
		Slider3->setMaximum(1000);
		label = new QLabel(centralwidget);
		label->setObjectName(QString::fromUtf8("label"));
		label->setGeometry(QRect(638, 22, 64, 17));
		label_2 = new QLabel(centralwidget);
		label_2->setObjectName(QString::fromUtf8("label_2"));
		label_2->setGeometry(QRect(638, 62, 64, 17));
		label_3 = new QLabel(centralwidget);
		label_3->setObjectName(QString::fromUtf8("label_3"));
		label_3->setGeometry(QRect(638, 104, 64, 17));
		pushButton = new QPushButton(centralwidget);
		pushButton->setObjectName(QString::fromUtf8("pushButton"));
		pushButton->setGeometry(QRect(650, 160, 131, 27));
		pushButton_2 = new QPushButton(centralwidget);
		pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
		pushButton_2->setGeometry(QRect(650, 190, 131, 27));
		MainWindow->setCentralWidget(centralwidget);
		menubar = new QMenuBar(MainWindow);
		menubar->setObjectName(QString::fromUtf8("menubar"));
		menubar->setGeometry(QRect(0, 0, 800, 25));
		MainWindow->setMenuBar(menubar);
		statusbar = new QStatusBar(MainWindow);
		statusbar->setObjectName(QString::fromUtf8("statusbar"));
		MainWindow->setStatusBar(statusbar);

		retranslateUi(MainWindow);


		QMetaObject::connectSlotsByName(MainWindow);
	} // setupUi

	void retranslateUi(QMainWindow *MainWindow)
	{
		MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
		label->setText(QApplication::translate("MainWindow", "Z MIN", 0, QApplication::UnicodeUTF8));
		label_2->setText(QApplication::translate("MainWindow", "Z MAX", 0, QApplication::UnicodeUTF8));
		label_3->setText(QApplication::translate("MainWindow", "P", 0, QApplication::UnicodeUTF8));
		pushButton->setText(QApplication::translate("MainWindow", "Show full", 0, QApplication::UnicodeUTF8));
		pushButton_2->setText(QApplication::translate("MainWindow", "Stuff", 0, QApplication::UnicodeUTF8));
	} // retranslateUi

};

namespace Ui {
class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VIEWER_SLIDER_H
