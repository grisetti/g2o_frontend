#include "Logger.h"

int main(int argc, char** argv){
  // creare la coda
  PrioritySynchronousDataQueue queue;
  
  // inizializzare Ros
  
  // creare i sensors
  
  // creare i sensor handler e passargli il puntatore alla coda &queue
  
  // init dei sensor handler (calibrazione)
  
  // registrazione su ROS delle callback dei sensor handler
  
  // inizio gestione della coda
  
  // lancio dei listener delle callback callback
  // ROS::spin()
}

void queueProcessing(PrioritySynchronousDataQueue * queue, std::string output_filename){
  int nextid = 1;
  StampedData * data;
  std::ofstream fout(output_filename.c_str());
  while(true){
    data = queue->front();
    if(data){	// queue is not empty
      if(data->timeStamp() < ros::Time::now().toSec() - ANCIENT_ENOUGH){	// data is old enough
	data = queue->front_and_pop();
	if(!data){continue;}	// someone else took away all the datas, go back to "while(true)"
	
	// get the trasformation at the data timestamp
	
	
	// write the vertex stuff to the output file
	fout << "VERTEX_SE3:QUAT " << nextid++ << std::endl;
	// write the data stuff on the next line
      }
    }
    else{	// queue is empty
      usleep(2e5);
    }
  }
}