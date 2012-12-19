#ifndef DM_MAIN_WINDOW_H
#define DM_MAIN_WINDOW_H

#include "ui_dm_base_main_window.h"

class DMMainWindow : public QMainWindow, public Ui::MainWindow {
 Q_OBJECT
 public:
  DMMainWindow(QWidget *parent = 0, Qt::WindowFlags flags = 0);
  ~DMMainWindow();
  bool* initialGuess() { return &_initialGuess; }
  bool* optimize() { return &_optimize; }
  int* step() { return &_step[0]; }
  float* points() { return &_points[0]; }
  float* normals() { return &_normals[0]; }
  float* covariances() { return &_covariances[0]; }
  float* correspondences() { return &_correspondences[0]; }

 private slots:
  // Checkbox slots.
  void slotStepEnabled(bool newState) { _step[0] = newState; }
  void slotPointsEnabled(bool newState) { _points[0] = newState; }
  void slotNormalsEnabled(bool newState) { _normals[0] = newState; }
  void slotCovariancesEnabled(bool newState) { _covariances[0] = newState; }
  void slotCorrespondencesEnabled(bool newState) { _correspondences[0] = newState; }
  // Spinbox slots.
  void slotStepChangeValue(int newValue) { _step[1] = newValue; }
  void slotPointsChangeValue(double newValue) { _points[1] = newValue; }
  void slotNormalsChangeValue(double newValue) { _normals[1] = newValue; }
  void slotCovariancesChangeValue(double newValue) { _covariances[1] = newValue; }
  void slotCorrespondencesChangeValue(double newValue) { _correspondences[1] = newValue; } 
  // Pushbuttons slots.
  void slotInitialGuessClicked() { _optimize = 0; _initialGuess = 1; }
  void slotOptimizeClicked() { _initialGuess = 0; _optimize = 1; }

 protected:
  bool _initialGuess;
  bool _optimize;
  int _step[2];
  float _points[2];
  float _normals[2];
  float _covariances[2];
  float _correspondences[2];
};

#endif
