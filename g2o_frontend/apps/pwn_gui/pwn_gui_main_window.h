#ifndef PWN_GUI_MAIN_WINDOW_H
#define PWN_GUI_MAIN_WINDOW_H

#include "pwn_gui_base_main_window.h"

class PWNGuiMainWindow : public QMainWindow, public Ui::PWNGuiBaseMainWindow {
 Q_OBJECT
 public:
  PWNGuiMainWindow(QWidget *parent = 0, Qt::WindowFlags flags = 0);
  ~PWNGuiMainWindow();
  bool* closing() { return &_closing; }
  bool* initialGuess() { return &_initialGuess; }
  bool* optimize() { return &_optimize; }
  bool* clearLast() { return &_clearLast; }
  bool* clearAll() { return &_clearAll; }
  bool* addCloud() { return &_addCloud; }
  bool* merge() { return &_merge; }
  bool* unmerge() { return &_unmerge; }
  bool* addAndOptimize() {return &_addAndOptimize;}
  int* stepByStep() { return &_stepByStep; }
  int* step() { return &_step[0]; }
  float* points() { return &_points[0]; }
  float* normals() { return &_normals[0]; }
  float* covariances() { return &_covariances[0]; }
  float* correspondences() { return &_correspondences[0]; }
  QGraphicsScene* scene0() { return _scene0; }
  QGraphicsScene* scene1() { return _scene1; }
  void closeEvent(QCloseEvent*) { _closing = 1; }
  QListWidgetItem* itemList() {return _itemList; }

 private slots:
  // Checkbox slots.
  void slotStepEnabled(bool newState) { _step[0] = newState; }
  void slotPointsEnabled(bool newState) { _points[0] = newState; }
  void slotNormalsEnabled(bool newState) { _normals[0] = newState; }
  void slotCovariancesEnabled(bool newState) { _covariances[0] = newState; }
  void slotCorrespondencesEnabled(bool newState) { _correspondences[0] = newState; }
  void slotStepByStepEnabled(bool newState) { _stepByStep = newState; }
  // Spinbox slots.
  void slotStepChangeValue(int newValue) { _step[1] = newValue; }
  void slotPointsChangeValue(double newValue) { _points[1] = newValue; }
  void slotNormalsChangeValue(double newValue) { _normals[1] = newValue; }
  void slotCovariancesChangeValue(double newValue) { _covariances[1] = newValue; }
  void slotCorrespondencesChangeValue(double newValue) { _correspondences[1] = newValue; } 
  // Pushbuttons slots.
  void slotInitialGuessClicked() { _initialGuess = 1; }
  void slotOptimizeClicked() { _optimize = 1; }
  void slotAddCloudClicked() { _addCloud = 1; }
  void slotClearLastClicked() { _clearLast = 1; }
  void slotClearAllClicked() { _clearAll = 1; }
  void slotMergeClicked() { _merge = 1; }
  void slotUnmergeClicked() { _unmerge = 1; }
  // List widget slot.
  void slotCloudSelected(QListWidgetItem *itemList_) { _itemList = itemList_; };

 protected:
  // Scenes for graphicsview widgets.
  QGraphicsScene *_scene0, *_scene1;
  // State variables.
  bool _closing;
  bool _initialGuess;
  bool _optimize;
  bool _addAndOptimize;
  bool _clearLast;
  bool _clearAll;
  bool _addCloud;
  bool _merge;
  bool _unmerge;
  int _stepByStep;
  int _step[2];
  float _points[2];
  float _normals[2];
  float _covariances[2];
  float _correspondences[2];
  QListWidgetItem *_itemList;
};

#endif
