#include "g2o_frontend/pwn_utils/pwn_file_format_converter.h"

int main() {
  string pwnFile = "out-00003.pwn";
  string pcdFile = "out-00003.pcd";
  
  // Convert pwn to pcd format
  pwnToPcd(pcdFile.c_str(), pwnFile.c_str());

  // Convert pcd to pwn format
  pcdToPwn("out-00003-copy.pwn", pcdFile.c_str());

  return 0;
}
