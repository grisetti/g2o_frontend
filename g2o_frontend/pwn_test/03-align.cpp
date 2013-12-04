#include <g2o/stuff/command_args.h>
#include <g2o_frontend/pwn_core/depthimage.h>
#include <g2o_frontend/pwn_core/depthimageconverterintegralimage.h>
#include <g2o_frontend/pwn_core/pinholepointprojector.h>
#include <g2o_frontend/pwn_core/statscalculatorintegralimage.h>
#include <g2o_frontend/pwn_core/informationmatrixcalculator.h>
#include <g2o_frontend/pwn_core/aligner.h>
#include <g2o_frontend/boss/serializer.h>
#include <g2o_frontend/boss/deserializer.h>
#include <string>
#include <cstdio>
using namespace std;
using namespace pwn;
using namespace boss;

bool loadFrameFromDepthImage(Frame* frame, const string& filename, DepthImageConverterIntegralImage* converter) {
  DepthImage depth;
  if (!depth.load(filename.c_str(), true)) {
    return false;
  }
  converter->compute(*frame, depth);
  return true;
}

int main(int argc, char** argv) {
  string outPwn;
  string configFile;
  bool saveBinary;
  int saveStep;

  vector<string> inFiles;

  g2o::CommandArgs arg;
  arg.param("outPwn", outPwn, "", "Filename of the output cloud");
  arg.param("saveStep", saveStep, 1, "Step parameter when saving");
  arg.param("saveBinary", saveBinary, true, "Use binary format when saving");
  arg.param("config", configFile, "", "BOSS configuration file for the objects");
  arg.parseArgs(argc, argv);

  int skip = 1;
  if (outPwn != "") skip++;
  if (saveStep != 1) skip++;
  if (configFile != "") skip++;
  int s = 0;
  for (int i = 0; i < argc; i++) {
    if (argv[i][0] != '-') {
      s++;
      if (s > skip) inFiles.push_back(argv[i]);
    }
  }

  PinholePointProjector* pinholeProjector = 0;
  StatsCalculatorIntegralImage* statsCalculator = 0;
  PointInformationMatrixCalculator* pimc = 0;
  NormalInformationMatrixCalculator* nimc = 0;
  DepthImageConverterIntegralImage* converter = 0;
  Aligner* aligner = 0;
  CorrespondenceFinder* corrFinder = 0;
  Linearizer* linearizer = 0;

  Deserializer des;
  Serializable* ser;
  des.setFilePath(configFile);
  while ((ser = des.readObject())) {
    if (dynamic_cast<PinholePointProjector*>(ser)) pinholeProjector = dynamic_cast<PinholePointProjector*>(ser);
    if (dynamic_cast<StatsCalculatorIntegralImage*>(ser)) statsCalculator = dynamic_cast<StatsCalculatorIntegralImage*>(ser);
    if (dynamic_cast<PointInformationMatrixCalculator*>(ser)) pimc = dynamic_cast<PointInformationMatrixCalculator*>(ser);
    if (dynamic_cast<NormalInformationMatrixCalculator*>(ser)) nimc = dynamic_cast<NormalInformationMatrixCalculator*>(ser);
    if (dynamic_cast<DepthImageConverterIntegralImage*>(ser)) converter = dynamic_cast<DepthImageConverterIntegralImage*>(ser);
    if (dynamic_cast<Aligner*>(ser)) aligner = dynamic_cast<Aligner*>(ser);
    if (dynamic_cast<CorrespondenceFinder*>(ser)) corrFinder = dynamic_cast<CorrespondenceFinder*>(ser);
    if (dynamic_cast<Linearizer*>(ser)) linearizer = dynamic_cast<Linearizer*>(ser);
  }
  if (!pinholeProjector) {
    printf("PinholePointProjector not found in config\n");
    pinholeProjector = new PinholePointProjector();
  }
  if (!statsCalculator) {
    printf("StatsCalculatorIntegralImage not found in config\n");
    statsCalculator = new StatsCalculatorIntegralImage();
  }
  if (!pimc) {
    printf("PointInformationMatrixCalculator not found in config\n");
    pimc = new PointInformationMatrixCalculator();
  }
  if (!nimc) {
    printf("NormalInformationMatrixCalculator not found in config\n");
    nimc = new NormalInformationMatrixCalculator();
  }
  if (!converter) {
    printf("DepthImageConverterIntegralImage not found in config\n");
    converter = new DepthImageConverterIntegralImage(pinholeProjector, statsCalculator, pimc, nimc);
  }
  if (!corrFinder) {
    printf("CorrespondenceFinder not found in config\n");
    corrFinder = new CorrespondenceFinder();
  }
  if (!linearizer) {
    printf("Linearizer not found in config\n");
    linearizer = new Linearizer();
  }
  if (!aligner) {
    printf("Aligner not found in config\n");
    aligner = new Aligner();
    aligner->setProjector(pinholeProjector);
    aligner->setCorrespondenceFinder(corrFinder);
    aligner->setLinearizer(linearizer);
  }


  printf("Reading frame 0...");
  Frame* frame0 = new Frame();
  if (!loadFrameFromDepthImage(frame0, inFiles[0], converter)) {
    fprintf(stderr, "Cannot open file '%s'.\n", inFiles[0].c_str());
    exit(-1);
  }
  frame0->save("reference.pwn", saveStep, saveBinary);
  printf("OK\n");

  Frame* frame = new Frame();
  for (size_t i = 1; i < inFiles.size(); i++)  {
    printf("Frame %zu: loading...", i);
    if (!loadFrameFromDepthImage(frame, inFiles[i], converter)) {
      fprintf(stderr, "Cannot open file '%s'.\n", inFiles[i].c_str());
      exit(-1);
    }
    frame->save("current.pwn", saveStep, saveBinary);
    aligner->setReferenceFrame(frame0);
    aligner->setCurrentFrame(frame);
    printf("aligning...");
    aligner->align();

    cout << "T: " << endl << aligner->T().matrix() << endl;
    cerr << "Error:   " << aligner->error() << endl;
    cerr << "Inliers: " << aligner->inliers() << endl;
    cerr << "Error/Inliers: ";
    if (aligner->inliers()) cerr << aligner->error()/aligner->inliers() << endl;
    else cerr << "NaN" << endl;

    printf("adding...");
    frame0->add(*frame, aligner->T());
    printf("OK\n");
  }

  std::cerr << "Saving final PWN in " << outPwn << std::endl;    
  frame0->save(outPwn.c_str(), saveStep, saveBinary);
  printf("OK\n");

  return 0;
}
