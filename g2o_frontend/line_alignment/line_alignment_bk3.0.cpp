#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include <g2o/core/hyper_graph.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"
#include "g2o_frontend/sensor_data/laser_robot_data.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o_frontend/ransac/alignment_line2d_linear.h"
#include "g2o_frontend/ransac/line2d_correspondence_validator.h"
#include "g2o_frontend/ransac/id_correspondence_validator.h"
#include "g2o_frontend/ransac/ransac.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include <boost/concept_check.hpp>
#include <boost/iterator/iterator_concepts.hpp>

#include "alignment_utils.h"

// #define DEBUG_MODE 1;

using namespace Eigen;
using namespace std;
using namespace g2o;
using namespace g2o_frontend;

bool updateVertexPointID(SparseOptimizer* graph, SparseOptimizer* graphline, VertexLine2D* vli, VertexLine2D* vlj) {
  //checking if the first extreme point of a line is a common vertex
  bool updated = false;
  VertexPointXY* vlip1 = dynamic_cast<VertexPointXY*>(graph->vertex(vli->p1Id));
  // 	VertexPointXY* vljp1 = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p1Id));
  OptimizableGraph::EdgeSet epoint1 = vlip1->edges();
  int count1 = 0;
  for (OptimizableGraph::EdgeSet::iterator it_vp1 = epoint1.begin(); it_vp1 != epoint1.end(); it_vp1++)
    {
      EdgeLine2DPointXY* elp1 = dynamic_cast<EdgeLine2DPointXY*>(*it_vp1);
      if(elp1) count1++;
    }
	
  cerr << "number of edges between point and lines: " << count1<< endl; 
  if(count1 > 1) {
    cerr << "This is a common vertex!!!" << endl;
    {
      //OptimizableGraph::EdgeSet esvli = vlip1->edges();
      //for (OptimizableGraph::EdgeSet::iterator it_vli = esvli.begin(); it_vli != esvli.end(); it_vli++)
      //{
      //	EdgeLine2DPointXY* elpoint = dynamic_cast<EdgeLine2DPointXY*>(*it_vli);
      //	if(!elpoint)
      //		continue;
      //	cout << "removing edge point-line2d" << endl;
      //	graphline->removeEdge(elpoint);
      //	
      //	EdgeSE2PointXY* eSEpoint = dynamic_cast<EdgeSE2PointXY*>(*it_vli);
      //	if(!eSEpoint)
      //		continue;
      //	cout << "removing edge point-se2" << endl;
      //	graphline->removeEdge(eSEpoint);
      //}
    }
    cout << "removing vertex..." << endl;

    graphline->removeVertex(vlip1);
    vli->p1Id = vlj->p1Id;
    updated = true;
  }
  else updated = false;
	
  //checking if the second extreme point of a line is a common vertex
  VertexPointXY* vlip2 = dynamic_cast<VertexPointXY*>(graph->vertex(vli->p2Id));
  //VertexPointXY* vljp2 = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p2Id));
  if(!vlip2)
    updated = false;
  OptimizableGraph::EdgeSet epoint2 = vlip2->edges();
  int count2 = 0;
  for (OptimizableGraph::EdgeSet::iterator it_vp2 = epoint2.begin(); it_vp2 != epoint2.end(); it_vp2++)
    {
      EdgeLine2DPointXY* elp2= dynamic_cast<EdgeLine2DPointXY*>(*it_vp2);
      if(elp2) count2++;
    }
	
  cerr << "number of edges between point 2 and lines: " << count2 << endl; 
  if(count2 > 1) {
    cerr << "The second is a common vertex!!!" << endl;
    if(vli->p2Id != vlj->p2Id){
      cerr << "ceccheccazzzu" << endl;
      vli->p2Id = vlj->p2Id;
      //cout << "removing second vertex..." << endl;
      //graphline->removeVertex(vlip2);
      updated = true;
    } 
  }
  else updated = false;
  return updated;
}


#if 0
// ofstream os1("Line1.dat");
// ofstream os2("Line2.dat");
// ofstream os2R("Line2Remapped.dat");
// ofstream osletto("LineAppenaLetto.dat");
#endif


int main(int argc, char**argv){
    
  string filename;
  string outfilename;
  g2o::CommandArgs arg;
  arg.param("o", outfilename, "outputGraph.g2o", "output file name");
  arg.paramLeftOver("graph-input", filename , "", "graph file which will be processed", true);
  arg.parseArgs(argc, argv);
  ofstream unmergedG2O("unmergedG2O.g2o");
  ofstream mergedG2O(outfilename.c_str());
    
  // graph construction
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
  SparseOptimizer * graph = new SparseOptimizer();
  graph->setAlgorithm(solverGauss);
  graph->load(filename.c_str());
  graph->save("inputGraph_alignment.g2o");
    
  // sort the vertices based on the id
  std::vector<int> vertexIds(graph->vertices().size());
  int k=0;
  for (OptimizableGraph::VertexIDMap::iterator it=graph->vertices().begin(); it!= graph->vertices().end(); it ++){
    vertexIds[k++] = (it->first);
  }
  std::sort(vertexIds.begin(), vertexIds.end());
  
  //just to load the correct library
  LaserRobotData data;
  
  Isometry2d odom0to1 = Isometry2d::Identity();
  VertexSE2* v_current = 0;
  VertexSE2* v_next = 0/*new VertexSE2*/;
  EdgeSE2* eSE2 = 0/*new EdgeSE2*/;
  OptimizableGraph::EdgeSet es;
  OptimizableGraph::EdgeSet es_next;
  OptimizableGraph::EdgeSet es_merged;
  LinesSet lvector;
  LinesSet lvector_next;
  LinesSet lvector_merged;
  LinesForMatching pairLinesSet;
  //  LinesForMatchingVector lineMatchingContainer;

  bool merdging = false;
  int lastID = (int)vertexIds.size()-1;

  //lines vector to be sorted
  std::vector<VertexLine2D*> lines1sort, lines2sort;

    
  //fixing the first vertex of the graph
  graph->vertex(0)->setFixed(1);
  
  /** Building the new graph with vertex payload aligned:
   * -for each vertex, reading its own payload and saving the payload of the following vertex,
   * -call findCorrespondances,
   * -call ransac to find the inliers,
   * -merging vertex of the inliers found
   **/
  cout << "\033[22;31;1m********************************START READING THE GRAPH********************************\033[0m" << endl; 
  cerr << "num vertici " << vertexIds.size() << endl;
  for (int i = 0; i<(int)vertexIds.size(); i++)
  {
//      cerr << "iteration: " << i << endl;
      OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);
      VertexSE2* v = dynamic_cast<VertexSE2*>(_v);
      if (!v)
          continue;
      cout << "\033[22;34;1m***********************************\033[0m" << endl;
      cout << "\033[22;34;1m***********NEW ITERATION***********\033[0m" << endl;
      cout << "\033[22;34;1m***********************************\033[0m" << endl;

      int vcurr_id = v->id();
      int next_id = -1;
      v_current = v;

      //debug: saving file.dat with lines
#if 0
      char nomefile[260];
      sprintf(nomefile,"Line_frame_%d.dat",i);
      ofstream osline(nomefile);
#endif
	
      cout << "###### edges from the current robot poses ######" << endl;
      es = v_current->edges();
      cout << "This vertex has " << es.size() << " edge." << endl;

      //TO BE CHECKED
      //saveOdom0to1(v_current, v_next, odom0to1, eSE2);
      //if (v_next) {
      //    cerr << "ahhhhhhhh v " << v_next << endl;
      //    cerr << "ahhhhhhhh e " << eSE2 << endl;
      //    next_id = v_next->id();
      //    es_next = v_next->edges();
      //    cout << "The next vertex has " << es_next.size() << " edge. " << next_id << endl;
      //    cout << endl << "***Extracting lines from CURRENT frame***" << endl;
      //    graphline->addEdge(eSE2);
      //} else {
      //    next_id = -1;
      //    es_next.clear();
      //    cout << "\033[22;32;1m!!!! There is no next vertex --> END OF GRAPH - size of edgeset of v_next:" << es_next.size() << "\033[0m"<< endl;
      //}
      // cerr << "edgeset del v " << countv << endl;

      for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++) {
          eSE2 = dynamic_cast<EdgeSE2*>(*itv);

          if (!eSE2)
              continue;

          VertexSE2* tmp0 = dynamic_cast<VertexSE2*>(eSE2->vertices()[0]);
          VertexSE2* tmp1 = dynamic_cast<VertexSE2*>(eSE2->vertices()[1]);
          cout << "- Odom edge from vertex  " << tmp0->id() << " to " << tmp1->id() << endl;
          if(tmp0->id() == vcurr_id)
          {
              odom0to1 = eSE2->measurement().toIsometry();
              // 	    Matrix3d odomNoise = odom0to1.matrix();
              cout << "Odometry transformation between the current vertex " << vcurr_id << " and the next one " << tmp1->id() << ":\n" << odom0to1.matrix() << endl;
              // 	    odom0to1.translation()*=2;
              // 	    odom0to1.rotation()*=2;
              // 	    cout << "Noise: \n" << odomNoise.matrix() << endl;
              v_next = dynamic_cast<VertexSE2*>(eSE2->vertices()[1]);
              next_id = v_next->id();
          } else {
              v_next = 0;
              next_id = -1;
              odom0to1 = Eigen::Isometry2d::Identity();
              cout << "###Skipping this edge (forward evaluation of the odometry)###" << endl;
          }
      }
      if(v_next) {
          es_next = v_next->edges();
          cout << "The next vertex has " << es_next.size() << " edge." << endl;
          cout << endl << "***Extracting lines from CURRENT frame***" << endl;

          //line extracted for the current vertex


          for (OptimizableGraph::EdgeSet::iterator itv = es.begin(); itv != es.end(); itv++)
          {
              EdgeSE2Line2D* el = dynamic_cast<EdgeSE2Line2D*>(*itv);
              if (!el)
                  continue;

              //VertexSE2* tmp = dynamic_cast<VertexSE2*>(el->vertices()[0]);
              VertexLine2D* vl = dynamic_cast<VertexLine2D*>(el->vertices()[1]);
              lines1sort.push_back(vl);
          }
          std::sort(lines1sort.begin(), lines1sort.end(),LineLengthComparator());

          //saving the lines from the current vertex
          int currvertexLine = 0;
          cerr << "ho merdgiato? " << merdging << endl;
          if(!merdging)
          {
              for (size_t i=0; i<lines1sort.size(); i++){
                  cout << "  !!! No merdging occurs yet: taking the original first frame !!!  " << endl;
                  VertexLine2D* vl = lines1sort[i];
                  Line2D li = vl->estimate();
                  //cout << "- Line " << currvertexLine << ": id "  << vl->id() << ": theta " << li(0) << ", rho " << li(1) /*<< ", estimate: " << vl->estimate().transpose()*/ << endl;
                  SE2 odomTransform(odom0to1);
                  Line2D tli = odomTransform*li;
                  lineOfVertex tliv;
                  tliv.line = tli;
                  tliv.vertex = v;
                  tliv.vline = vl;
                  // 		cout << ">>> Line transformed in the next frame id " << tliv.vline->id() << ": theta " << tliv.line(0) << ", rho " << tliv.line(1) << endl;
                  lvector.push_back(tliv);
                  currvertexLine++;
#if 0
                  Vector2d line1 = Vector2d(vl->estimate());
                  VertexPointXY* vpl1_1 = dynamic_cast<VertexPointXY*>(graph->vertex(vl->p1Id));
                  VertexPointXY* vpl2_1 = dynamic_cast<VertexPointXY*>(graph->vertex(vl->p2Id));
                  Vector2d nline1(cos(line1(0)), sin(line1(0)));
                  Vector2d pmiddle1 = nline1*line1(1);
                  Vector2d t2(-nline1.y(), nline1.x());
                  double l1_1,l2_1 = 10;
                  l1_1 = t2.dot(vpl1_1->estimate() - pmiddle1);
                  l2_1 = t2.dot(vpl2_1->estimate() - pmiddle1);
                  Vector2d p1line1 = pmiddle1 + t2*l1_1;
                  Vector2d p2line1 = pmiddle1 + t2*l2_1;
                  osline << p1line1.transpose() << endl;
                  osline << p2line1.transpose() << endl;
                  osline << endl;
                  osline << endl;
                  osline.flush();
#endif  
              }
              cout << "Saved Lines of the current vertex.. size is: " << lvector.size() << endl;

          } else if(merdging) {
              cout << "  !!! Merdging happened --> Using " << lvector_merged.size() << " line vertex already aligned" << endl;
              SE2 odomTransform(odom0to1);
              for(int i = 0; i < (int)lvector_merged.size(); i++){
                  lineOfVertex livtmp = lvector_merged[i];
                  Line2D tli = odomTransform*(livtmp.line);
                  lineOfVertex tliv;
                  tliv.line = tli;
                  tliv.vertex = livtmp.vertex;
                  tliv.vline = livtmp.vline;
                  // 	      cout << ">>> Merged line transformed in the next frame id " << tliv.vline->id() << ": theta " << tliv.line(0) << ", rho " << tliv.line(1) << endl;
                  lvector.push_back(tliv);
              }
              cout << "Saved Lines previously merged.. size is: " << lvector.size() << endl;
              merdging = false;
          }

          //saving the lines of the next vertex, to create the correspondences for ransac
          int nextvertexLine = 0;
          cout << endl <<  "***Extracting lines from NEXT frame***" << endl;

          // ofstream osletto("Line1AppenaLetto.dat");

          for (OptimizableGraph::EdgeSet::iterator itv_next = es_next.begin(); itv_next != es_next.end(); itv_next++)
          {
              EdgeSE2Line2D* el_next = dynamic_cast<EdgeSE2Line2D*>(*itv_next);
              if (!el_next)
                  continue;

//              VertexSE2* tmp0_next = dynamic_cast<VertexSE2*>(el_next->vertices()[0]);
              VertexLine2D* vl_next = dynamic_cast<VertexLine2D*>(el_next->vertices()[1]);
              lines2sort.push_back(vl_next);
          }
          std::sort(lines2sort.begin(), lines2sort.end(),LineLengthComparator());

          for (size_t j=0; j<lines2sort.size(); j++)
          {
              VertexLine2D* vl_next=lines2sort[j];
              Line2D li_next = vl_next->estimate();
              lineOfVertex liv_next;
              liv_next.line = li_next;
              liv_next.vertex = v_next;
              liv_next.vline = vl_next;
              // 		cout << "- Line of the next vertex " << nextvertexLine << ": id "  << vl_next->id() << ": theta " << liv_next.line(0) << ", rho " << liv_next.line(1) << endl;
              lvector_next.push_back(liv_next);
              nextvertexLine++;
#if 0
              Vector2d line1 = Vector2d(vl_next->estimate());
              VertexPointXY* vpl1_1 = dynamic_cast<VertexPointXY*>(graph->vertex(vl_next->p1Id));
              VertexPointXY* vpl2_1 = dynamic_cast<VertexPointXY*>(graph->vertex(vl_next->p2Id));
              Vector2d nline1(cos(line1(0)), sin(line1(0)));
              Vector2d pmiddle1 = nline1*line1(1);
              Vector2d t2(-nline1.y(), nline1.x());
              double l1_1,l2_1 = 10;
              l1_1 = t2.dot(vpl1_1->estimate() - pmiddle1);
              l2_1 = t2.dot(vpl2_1->estimate() - pmiddle1);
              Vector2d p1line1 = pmiddle1 + t2*l1_1;
              Vector2d p2line1 = pmiddle1 + t2*l2_1;
              osline << p1line1.transpose() << endl;
              osline << p2line1.transpose() << endl;
              osline << endl;
              osline << endl;
              osline.flush();
#endif
          }
          cout << "Saved lines of the next vertex, size is:" << lvector_next.size() << endl;


          pairLinesSet = make_pair(lvector, lvector_next);
          cout << endl << " ### iteration "  << i << ", SIZE of the pair of lines sets: " << pairLinesSet.first.size() << ", " << pairLinesSet.second.size() << endl << endl;

          //debug
          // for(size_t uno = 0; uno < (lineMatchingContainer[i].first).size(); uno++) {
          // cout << "primo set:size " << (lineMatchingContainer[i].first).size() << "\n" << (lineMatchingContainer[i].first)[uno].line << endl << endl;
          // }
          // for(size_t due = 0; due < (lineMatchingContainer[i].second).size(); due++){
          // cout << "secondo set:size " << (lineMatchingContainer[i].second).size() << "\n" << (lineMatchingContainer[i].second)[due].line << endl << endl;
          // }

          cout << "\033[22;34;1m*********END OF ITERATION**********\033[0m" << endl << endl;

      } else if(!v_next){

          //checking if we reach the end of the graph
          es_next.clear();
          cout << "\033[22;32;1m!!!! There is no next vertex --> END OF GRAPH - size of edgeset of v_next:" << es_next.size() << "\033[0m"<< endl;
          cerr << "end of the graph" << endl;
          continue;
      }
	
      /// calling find correspondances
      cout << endl << "\033[22;31;1m....Starting correspondences finder..\033[0m" << endl;
      
      LineCorrs currCorrs;
      bool resultCorrespondances = findCorrespondences(currCorrs, pairLinesSet);
      cout << "\033[22;31;1m....End of correspondances finder..\033[0m" << endl << endl;
      if(resultCorrespondances)
      {
          /// call aligner
          cout << endl << "\033[22;31;1m**********************STARTING ALIGNMENT ALGORITHM: ITERATION " << /*c*/i << "**********************\033[0m " << endl << endl;

          CorrespondenceVector correspondences;
          IndexVector indices(currCorrs.size(), 0);

          LinesSet s1 = pairLinesSet.first;
          LinesSet s2 = pairLinesSet.second;

          //debug
#if 0
          ofstream osporca2_("Line2_.dat");
          for(int h = 0; h < s2.size(); h++){
              Vector2d line2 = Vector2d(s2[h].line);
              VertexPointXY* vpl1_2_ = dynamic_cast<VertexPointXY*>(graph->vertex(s2[h].vline->p1Id));
              VertexPointXY* vpl2_2_ = dynamic_cast<VertexPointXY*>(graph->vertex(s2[h].vline->p2Id));
              Vector2d nline2(cos(line2(0)), sin(line2(0)));
              Vector2d pmiddle2_ = nline2*line2(1);
              Vector2d t2(-nline2.y(), nline2.x());
              double l1_2,l2_2 = 10;
              l1_2 = t2.dot(vpl1_2_->estimate() - pmiddle2_);
              l2_2 = t2.dot(vpl2_2_->estimate() - pmiddle2_);
              Vector2d p1line2 = pmiddle2_ + t2*l1_2;
              Vector2d p2line2 = pmiddle2_ + t2*l2_2;
              osporca2_ << p1line2.transpose() << endl;
              osporca2_ << p2line2.transpose() << endl;
              osporca2_ << endl;
              osporca2_ << endl;
              osporca2_.flush();
          }
          ofstream osporca3_("Line1_.dat");
          for(int f = 0; f < s1.size(); f++){
              Vector2d line1 = Vector2d(s1[f].line);
              VertexPointXY* vpl1_1 = dynamic_cast<VertexPointXY*>(graph->vertex(s1[f].vline->p1Id));
              VertexPointXY* vpl2_1 = dynamic_cast<VertexPointXY*>(graph->vertex(s1[f].vline->p2Id));
              Vector2d nline1(cos(line1(0)), sin(line1(0)));
              Vector2d pmiddle1 = nline1*line1(1);
              Vector2d t1(-nline1.y(), nline1.x());
              double l1_1,l2_1 = 10;
              l1_1 = t1.dot(vpl1_1->estimate() - pmiddle1);
              l2_1 = t1.dot(vpl2_1->estimate() - pmiddle1);
              Vector2d p1line1 = pmiddle1 + t1*l1_1;
              Vector2d p2line1 = pmiddle1 + t1*l2_1;
              osporca3_ << p1line1.transpose() << endl;
              osporca3_ << p2line1.transpose() << endl;
              osporca3_ << endl;
              osporca3_ << endl;
              osporca3_.flush();
          }
#endif

          cout << "!!! Size of correspondances founded for the current pair of vertex: " << currCorrs.size() << endl;

          // ofstream osporca4("Line2CurrCorr.dat");

          for (int ci = 0; ci < (int)currCorrs.size(); ci++)
          {
              cerr << "Correspondances position in lines sets: "  <<currCorrs[ci].lid1 << ", " << currCorrs[ci].lid2 << ", with error:  " << currCorrs[ci].error << endl;
              //debug
              // 		int vlid1 = -1, vlid2 = -1;
              // 		cerr << "ID vertex 1: [" << s1[currCorrs[ci].lid1].vline->id() << "] - ID vertex 2: [" << s2[currCorrs[ci].lid2].vline->id() << "]" << endl;
              // 		vlid1 = graph->vertex(s1[currCorrs[ci].lid1].vline->id())->id();
              // 		vlid2 = graph->vertex(s2[currCorrs[ci].lid2].vline->id())->id();
              // 		cerr << "ID vertex 1: [" << vlid1 << "] - ID vertex 2: [" << vlid2 << "]" << endl;
              EdgeLine2D* eline = new EdgeLine2D;
              VertexLine2D* vli = dynamic_cast<VertexLine2D*>(graph->vertex(s1[currCorrs[ci].lid1].vline->id()));
              Eigen::Vector2d p11=dynamic_cast<const VertexPointXY*>(graph->vertex(vli->p1Id))->estimate();
              Eigen::Vector2d p12=dynamic_cast<const VertexPointXY*>(graph->vertex(vli->p2Id))->estimate();
              double d1=(p11-p12).squaredNorm();

              VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(graph->vertex(s2[currCorrs[ci].lid2].vline->id()));
              Eigen::Vector2d p21=dynamic_cast<const VertexPointXY*>(graph->vertex(vlj->p1Id))->estimate();
              Eigen::Vector2d p22=dynamic_cast<const VertexPointXY*>(graph->vertex(vlj->p2Id))->estimate();
              double d2=(p21-p22).squaredNorm();
              double diff=1./(1+fabs(d1-d2));

              //debug
#if 0
              Vector2d line2 = Vector2d(vlj->estimate());
              VertexPointXY* vpl1_2_ = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p1Id));
              VertexPointXY* vpl2_2_ = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p2Id));
              Vector2d nline2(cos(line2(0)), sin(line2(0)));
              Vector2d pmiddle2_ = nline2*line2(1);
              Vector2d t2(-nline2.y(), nline2.x());
              double l1_2,l2_2 = 10;
              l1_2 = t2.dot(vpl1_2_->estimate() - pmiddle2_);
              l2_2 = t2.dot(vpl2_2_->estimate() - pmiddle2_);
              Vector2d p1line2 = pmiddle2_ + t2*l1_2;
              Vector2d p2line2 = pmiddle2_ + t2*l2_2;
              osporca3 << p1line2.transpose() << endl;
              osporca3 << p2line2.transpose() << endl;
              osporca3 << endl;
              osporca3 << endl;
              osporca3.flush();
#endif
              eline->setVertex(0,vli);
              eline->setVertex(1,vlj);
              Matrix2d info;
              info << 1000, 0, 0, 10;
              eline->setInformation(info);
              eline->setMeasurement(Eigen::Vector2d::Zero());
              Correspondence c(eline,diff);
              correspondences.push_back(c);
              indices[ci]=ci;

              //debug
              // 	      	for(int i = 0; i < s1.size(); i++){
              // 	      		if(s1[i].vline->id() == currCorrs[ci].lid1){
              // 	      			vlid1 = s1[i].vline->id();
              // 	      			cout << "[1] id vertex: " << vlid1 << endl;
              // 	      		}
              // 	      	}
              // 	      	for(int j = 0; j < s2.size(); j++){
              // 	      		if(s1[j].vline->id() == currCorrs[ci].lid1){
              // 	      			vlid2 = s2[j].vline->id();
              // 	      			cout << "[2] id vertex: " << vlid2 << endl;
              // 	      		}
              // 	      	}
          }
          cerr << "size of correspondances vector: " << correspondences.size() << endl << endl;

          /// call ransac
          CorrespondenceValidatorPtrVector validators;
          Line2DCorrespondenceValidator<VertexLine2D>* val1 = new Line2DCorrespondenceValidator<VertexLine2D>();
          val1->setIntraFrameDistanceDifference(.1);
          val1->setIntraFrameMinimalDistance(.25);
          validators.push_back(val1);
          IdCorrespondenceValidator* val2 = new IdCorrespondenceValidator(2);
          validators.push_back(val2);

          SE2 t0(odom0to1);
          RansacLine2DLinear::TransformType transform = t0;
          std::vector<int> inliers;
          int iterations = 1000;
          float inliersThreshold = .3;
          float inliersStopFraction = .5;
          vector<double> err;
          ScopeTime t("ransac aligned");
          bool resultRansac = ransacExec(validators, correspondences, inliers, transform, iterations, inliersThreshold, inliersStopFraction, err, false, 1);

          if(resultRansac)
          {
              cerr << "\033[22;32;1m***********TRANSFORM FOUND!***********\033[0m" << endl;
              Isometry2d res = transform.toIsometry();
              Isometry2d _t0 = t0.toIsometry();
              //cerr << endl;
              //cerr << "ground truth vector: " <<endl;
              //cerr << t2v_2d(_t0) << endl;
              //cerr << "ground truth: " <<endl;
              //cerr << _t0.matrix() << endl;
              cerr << endl;
              cerr << "transform found vector: " << endl;
              cerr << t2v_2d(res) << endl;
              //cerr << "transform found: " <<endl;
              //cerr << res.matrix() << endl;
              cerr << endl;
              cerr << "transform error vector: " << endl;
              cerr << t2v_2d(_t0*res) << endl;
              //cerr << "transform error: " << endl;
              //cerr << (_t0*res).matrix() << endl;

              cout << "Erros size: " << err.size() << endl;
              for (int h = 0; h < (int)err.size(); h++)
              {
                  double erri = err[h];
                  cout << "error of " << h << "-th correspondance: " << erri << endl;
              }
              cout << "Inliers: (size is " << inliers.size() << ")" << endl;
              for(int g = 0; g < (int)inliers.size(); g++)
              {
                  double index = inliers[g];
                  cout << "Inliers: " << index << "-th correspondance: " << " with error: " << err[index] << endl;
              }

              //updating the value of the second vertex pose and his own line measurements
              SE2 newpose = v_next->estimate()*transform.inverse();
              cerr << "vecchia posa: \n" << v_next->estimate().toIsometry().matrix() << endl;
              v_next->setEstimate(newpose);
              cerr << "nuova posa: \n" << v_next->estimate().toIsometry().matrix() << endl;

              for (OptimizableGraph::EdgeSet::iterator itv_next = es_next.begin(); itv_next != es_next.end(); itv_next++)
              {
                  EdgeSE2Line2D* el_next = dynamic_cast<EdgeSE2Line2D*>(*itv_next);
                  if (!el_next) continue;

                  VertexLine2D* vl_next = dynamic_cast<VertexLine2D*>(el_next->vertices()[1]);
                  if(!vl_next) continue;

                  Line2D newli_next = transform.inverse()*vl_next->estimate();
                  vl_next->setEstimate(newli_next);
              }

              //debug: plotting lines frame i and lines frame j remapped with the transform found
#if 1
              ofstream os1("Line1.dat");
              for(int f = 0; f < (int)s1.size(); f++)
              {
                  Vector2d line1 = Vector2d(s1[f].line);
                  VertexPointXY* vpl1_1 = dynamic_cast<VertexPointXY*>(graph->vertex(s1[f].vline->p1Id));
                  VertexPointXY* vpl2_1 = dynamic_cast<VertexPointXY*>(graph->vertex(s1[f].vline->p2Id));
                  Vector2d nline1(cos(line1(0)), sin(line1(0)));
                  Vector2d pmiddle1 = nline1*line1(1);
                  Vector2d t1(-nline1.y(), nline1.x());
                  double l1_1,l2_1 = 10;
                  l1_1 = t1.dot(vpl1_1->estimate()-pmiddle1);
                  l2_1 = t1.dot(vpl2_1->estimate()-pmiddle1);
                  Vector2d p1line1 = pmiddle1 + t1*l1_1;
                  Vector2d p2line1 = pmiddle1 + t1*l2_1;
                  os1 << p1line1.transpose() << endl;
                  os1 << p2line1.transpose() << endl;
                  os1 << endl;
                  os1 << endl;
                  os1.flush();
              }

              ofstream os1c("Line1CurrCorr.dat");
              for (int ci = 0; ci < (int)currCorrs.size(); ci++)
              {
                  VertexLine2D* vli = dynamic_cast<VertexLine2D*>(graph->vertex(s1[currCorrs[ci].lid1].vline->id()));
                  Vector2d line1 = Vector2d(vli->estimate());
                  VertexPointXY* vpl1_1 = dynamic_cast<VertexPointXY*>(graph->vertex(s1[currCorrs[ci].lid1].vline->p1Id));
                  VertexPointXY* vpl2_1 = dynamic_cast<VertexPointXY*>(graph->vertex(s1[currCorrs[ci].lid1].vline->p2Id));
                  Vector2d nline1(cos(line1(0)), sin(line1(0)));
                  Vector2d pmiddle1 = nline1*line1(1);
                  Vector2d t1(-nline1.y(), nline1.x());
                  double l1_1,l2_1 = 10;
                  l1_1 = t1.dot(vpl1_1->estimate()-pmiddle1);
                  l2_1 = t1.dot(vpl2_1->estimate()-pmiddle1);
                  Vector2d p1line1 = pmiddle1 + t1*l1_1;
                  Vector2d p2line1 = pmiddle1 + t1*l2_1;
                  os1c << p1line1.transpose() << endl;
                  os1c << p2line1.transpose() << endl;
                  os1c << endl;
                  os1c << endl;
                  os1c.flush();
              }

              ofstream os2("Line2.dat");
              ofstream os2R("Line2Remapped.dat");
              for(int g = 0; g < (int)s2.size(); g++)
              {
                  Vector2d line2 = Vector2d(s2[g].line);
                  VertexPointXY* vpl1_2_ = dynamic_cast<VertexPointXY*>(graph->vertex(s2[g].vline->p1Id));
                  VertexPointXY* vpl2_2_ = dynamic_cast<VertexPointXY*>(graph->vertex(s2[g].vline->p2Id));
                  Vector2d nline2(cos(line2(0)), sin(line2(0)));
                  Vector2d pmiddle2_ = nline2*line2(1);
                  Vector2d t2(-nline2.y(), nline2.x());
                  double l1_2,l2_2 = 10;
                  l1_2 = t2.dot(vpl1_2_->estimate() - pmiddle2_);
                  l2_2 = t2.dot(vpl2_2_->estimate() - pmiddle2_);
                  Vector2d p1line2 = pmiddle2_ + t2*l1_2;
                  Vector2d p2line2 = pmiddle2_ + t2*l2_2;
                  os2 << p1line2.transpose() << endl;
                  os2 << p2line2.transpose() << endl;
                  os2 << endl;
                  os2 << endl;
                  os2.flush();

                  //taking the set of line to be trasformed
                  Vector2d line2Remapped = Vector2d(transform*s2[g].line);
                  VertexPointXY* vpl1_2 = dynamic_cast<VertexPointXY*>(graph->vertex(s2[g].vline->p1Id));
                  VertexPointXY* vpl2_2 = dynamic_cast<VertexPointXY*>(graph->vertex(s2[g].vline->p2Id));
                  Vector2d vpl1_2R = transform*(vpl1_2->estimate());
                  Vector2d vpl2_2R = transform*(vpl2_2->estimate());
                  Vector2d nline2R(cos(line2Remapped(0)), sin(line2Remapped(0)));
                  Vector2d pmiddle2 = nline2R*line2Remapped(1);
                  Vector2d t2R(-nline2R.y(), nline2R.x());
                  double l1_2R,l2_2R = 10;
                  l1_2R = t2R.dot(vpl1_2R - pmiddle2);
                  l2_2R = t2R.dot(vpl2_2R - pmiddle2);
                  Vector2d p1line2R = pmiddle2 + t2R*l1_2R;
                  Vector2d p2line2R = pmiddle2 + t2R*l2_2R;
                  os2R << p1line2R.transpose() << endl;
                  os2R << p2line2R.transpose() << endl;
                  os2R << endl;
                  os2R << endl;
                  os2R.flush();
              }

              ofstream os2c("Line2CurrCorr.dat");
              for (int ci = 0; ci < (int)currCorrs.size(); ci++)
              {
                  VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(graph->vertex(s2[currCorrs[ci].lid2].vline->id()));
                  Vector2d line2 = Vector2d(vlj->estimate());
                  VertexPointXY* vpl1_2 = dynamic_cast<VertexPointXY*>(graph->vertex(s2[currCorrs[ci].lid2].vline->p1Id));
                  VertexPointXY* vpl2_2 = dynamic_cast<VertexPointXY*>(graph->vertex(s2[currCorrs[ci].lid2].vline->p2Id));
                  Vector2d nline2(cos(line2(0)), sin(line2(0)));
                  Vector2d pmiddle2 = nline2*line2(1);
                  Vector2d t1(-nline2.y(), nline2.x());
                  double l1_1,l2_1 = 10;
                  l1_1 = t1.dot(vpl1_2->estimate()-pmiddle2);
                  l2_1 = t1.dot(vpl2_2->estimate()-pmiddle2);
                  Vector2d p1line2 = pmiddle2 + t1*l1_1;
                  Vector2d p2line2 = pmiddle2 + t1*l2_1;
                  os2c << p1line2.transpose() << endl;
                  os2c << p2line2.transpose() << endl;
                  os2c << endl;
                  os2c << endl;
                  os2c.flush();
              }
#endif
              {// if not using "mergeLineVertex(graph, graphline, vlj, vli);" book keeping (updating the vertex point id of the correspondences for common vertex)
                  //              for (int ci = 0; ci < currCorrs.size(); ci++)
                  //              {
                  //                  VertexLine2D* vli = dynamic_cast<VertexLine2D*>(graph->vertex(s1[currCorrs[ci].lid1].vline->id()));
                  //                  VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(graph->vertex(s2[currCorrs[ci].lid2].vline->id()));
                  //                  updateVertexPointID(graph, graphline, vli, vlj);
                  //              }
              }

              //saving the graph before merdging!
              graph->save(unmergedG2O);
              unmergedG2O.close();

              ///merging vertexes and lines (inliers set)
              cout << endl << "\033[22;34;1m*****MERGING STUFF******\033[0m " << endl << endl;

              for (int ci = 0; ci < (int)inliers.size(); ci++)
              {

                  //TODO to be UNCOMMENT


                  //              double inliersIndex = inliers[ci];
                  //              VertexLine2D* vli = dynamic_cast<VertexLine2D*>(graph->vertex(s1[currCorrs[/*ci*/inliersIndex].lid1].vline->id()));
                  //              VertexLine2D* vlj = dynamic_cast<VertexLine2D*>(graph->vertex(s2[currCorrs[/*ci*/inliersIndex].lid2].vline->id()));
                  //              cout << "Line to be merged: " << endl;
                  //              cout << "[Frame i] line " << vli->id() << " - [Frame j] line " << vlj->id() << endl;
                  //              merdging = mergeLineVertex(graph, vlj, vli);
                  //              cout << endl << " \033[22;32;1miteration " << ci  << " -- Lines merged? " << merdging << "\033[0m" << endl;
                  //              cout << endl;

                  {//mine merging implementation
                      // 					VertexPointXY* vlip1 = dynamic_cast<VertexPointXY*>(graph->vertex(vli->p1Id));
                      // 					VertexPointXY* vljp1 = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p1Id));
                      //
                      // 					if(vlip1->id() != vljp1->id()) {
                      // 						bool resultmergePoint1 = graphline->mergeVertices(vljp1, vlip1 ,true);
                      // 						cout << "merging point1: " << resultmergePoint1 << endl;
                      // 					}
                      //
                      // 					VertexPointXY* vlip2 = dynamic_cast<VertexPointXY*>(graph->vertex(vli->p2Id));
                      // 					VertexPointXY* vljp2 = dynamic_cast<VertexPointXY*>(graph->vertex(vlj->p2Id));
                      // 					if(!vlip2 && !vljp2) cerr << "che cribbio succede???" << endl;
                      // 					if(vlip2 || vljp2) {
                      // 						if(vlip2 && !vljp2){
                      // 							cerr << "imposible ploplio --> agiolnelei indice" << endl;
                      // 							vlj->p2Id = vlip2->id();
                      // 						} else if(!vlip2 && vljp2) {
                      // 							cerr << "nun faccio una mazza" << endl;
                      // 						} else if(vlip2 && vljp2 && vlip2->id() != vljp2->id()){
                      // 							bool resultmergePoint2 = graphline->mergeVertices(vljp2, vlip2 ,true);
                      // 							cout << "merging point2: " << resultmergePoint2 << endl;
                      // 						}
                      // 					}
                      // 					bool resultmergeLine = graphline->mergeVertices(vlj, vli ,true);
                      // 					cout << "merging line: " << resultmergeLine << endl;
                  }
              }
              //          graph->initializeOptimization();
              //          graph->optimize(10);

              //saving the new line vertex already aligned
              if(merdging && v_next->id() != lastID)
              {
                  lvector_merged.clear();
                  VertexSE2* lastVertexAligned = dynamic_cast<VertexSE2*>(graph->vertex(s2[0].vertex->id()));//ugly! in s2 there are the line of the second vertex used in ransac
                  es_merged = lastVertexAligned->edges();
                  cout << endl << "***Saving the last line vertex already aligned into lvector for the next findCorrespondances iteration: >> id vertex: *** " << lastVertexAligned->id() << endl;
                  for (OptimizableGraph::EdgeSet::iterator itvm = es_merged.begin(); itvm != es_merged.end(); itvm++)
                  {
                      EdgeSE2Line2D* elm = dynamic_cast<EdgeSE2Line2D*>(*itvm);
                      if (!elm)
                          continue;

                      VertexSE2* tmp = dynamic_cast<VertexSE2*>(elm->vertices()[0]);
                      VertexLine2D* vlm = dynamic_cast<VertexLine2D*>(elm->vertices()[1]);

                      //saving the lines adding vertexline to the new graph
                      if (tmp->id() == lastVertexAligned->id())
                      {
                          Line2D lim = vlm->estimate();
                          cout << "- Line id "  << vlm->id() << ": theta " << lim(0) << ", rho " << lim(1) /*<< ", estimate: " << vl->estimate().transpose()*/ << endl;
                          lineOfVertex livm;
                          livm.line = lim;
                          livm.vertex = lastVertexAligned;
                          livm.vline = vlm;
                          lvector_merged.push_back(livm);
                      }
                  }
                  cerr << "After Merdging:  (" << merdging << ") -- lvector aligned size: "  << lvector_merged.size() << endl;
              }
              cout << endl << "\033[22;31;1m**********************END OF ALIGNMENT ALGORITHM: ITERATION " << /*c*/i << "**********************\033[0m" << endl << endl;
          }
          cout << "out of result ransac" << endl;
      }
      cout << "out of result correspondances" << endl;
      lvector.clear();
      lvector_next.clear();
  }
    
  cout << endl << "\033[22;31;1m********************************END READING THE GRAPH********************************\033[0m" << endl << endl;
  cout << endl;
  // 	delete(v_next);
  // 	delete(eSE2);
  lvector_merged.clear();
  cout << "vectors of lines at the end: current" << lvector.size() << ", next " << lvector_next.size() << ", merged " << lvector_merged.size() << endl;

  cout << "...saving merged graph in " << outfilename.c_str() << endl;
  graph->save(mergedG2O);
  mergedG2O.close();
  return (0);
}

