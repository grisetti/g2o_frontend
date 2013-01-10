#include "pwn_gui_main_window.h"

PWNGuiMainWindow::PWNGuiMainWindow(QWidget *parent, Qt::WindowFlags flags) : QMainWindow(parent, flags) {
  setupUi(this);

  // Create scenes for graphicsview widgets and set them.
  _scene0 = new QGraphicsScene();
  _scene1 = new QGraphicsScene();
  graphicsView1_2d->setScene(_scene0);
  graphicsView2_2d->setScene(_scene1);
  
  // Variable initialization
  _closing = 0;
  _initialGuess = 1;
  _optimize = 0;
  _addCloud = 0;
  _stepByStep = 0;
  _step[0] = 1; _step[1] = 1;
  _points[0] = 1.0f; _points[1] = 1.0f;
  _normals[0] = 0.0f; _normals[1] = 0.0f;
  _covariances[0] = 0.0f; _covariances[1] = 0.0f;
  _correspondences[0] = 0.0f; _correspondences[1] = 0.0f;
  _itemList = 0;

  // Signals/Slots checkbox connections.
  QObject::connect(checkBox_step, SIGNAL(toggled(bool)),
		   this, SLOT(slotStepEnabled(bool)));
  QObject::connect(checkBox_points, SIGNAL(toggled(bool)),
		   this, SLOT(slotPointsEnabled(bool)));
  QObject::connect(checkBox_normals, SIGNAL(toggled(bool)),
		   this, SLOT(slotNormalsEnabled(bool)));
  QObject::connect(checkBox_covariances, SIGNAL(toggled(bool)),
		   this, SLOT(slotCovariancesEnabled(bool)));
  QObject::connect(checkBox_correspondences, SIGNAL(toggled(bool)),
		   this, SLOT(slotCorrespondencesEnabled(bool)));
  QObject::connect(checkBox_step_by_step, SIGNAL(toggled(bool)),
		   this, SLOT(slotStepByStepEnabled(bool)));

  QObject::connect(spinBox_step, SIGNAL(valueChanged(int)),
		   this, SLOT(slotStepChangeValue(int)));
  QObject::connect(doubleSpinBox_points, SIGNAL(valueChanged(double)),
		   this, SLOT(slotPointsChangeValue(double)));
  QObject::connect(doubleSpinBox_normals, SIGNAL(valueChanged(double)),
		   this, SLOT(slotNormalsChangeValue(double)));
  QObject::connect(doubleSpinBox_covariances, SIGNAL(valueChanged(double)),
		   this, SLOT(slotCovariancesChangeValue(double)));
  QObject::connect(doubleSpinBox_correspondences, SIGNAL(valueChanged(double)),
		   this, SLOT(slotCorrespondencesChangeValue(double)));

  // Signals/Slots pushbuttons connections.
  QObject::connect(pushButton_initial_guess, SIGNAL(clicked()),
		   this, SLOT(slotInitialGuessClicked()));
  QObject::connect(pushButton_optimize, SIGNAL(clicked()),
		   this, SLOT(slotOptimizeClicked()));
  QObject::connect(pushButton_add_cloud, SIGNAL(clicked()),
		   this, SLOT(slotAddCloudClicked()));

  // Signals/Slots list widget connections.
  QObject::connect(listWidget, SIGNAL(itemClicked(QListWidgetItem*)),
		   this, SLOT(slotCloudSelected(QListWidgetItem*)));
}

PWNGuiMainWindow::~PWNGuiMainWindow() {}
