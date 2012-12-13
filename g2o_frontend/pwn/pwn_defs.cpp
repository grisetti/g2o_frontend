#include "pwn_defs.h"

SVDMatrix3f::SVDMatrix3f() {
  isometry.setIdentity();
  lambda.setZero();
}
