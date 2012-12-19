#include "dm_main_window.h"

DMMainWindow::DMMainWindow(QWidget *parent, Qt::WindowFlags flags) : QMainWindow(parent, flags) {
  setupUi(this);

  // Variable initialization.
  _step[0] = 0; _step[1] = 1;
  _points[0] = 0.0f; _points[1] = 0.0f;
  _normals[0] = 0.0f; _normals[1] = 0.0f;
  _covariances[0] = 0.0f; _covariances[1] = 0.0f;
  _correspondences[0] = 0.0f; _correspondences[1] = 0.0f;

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
  // Signals/Slots spinbox connections.
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
}

DMMainWindow::~DMMainWindow() {}
