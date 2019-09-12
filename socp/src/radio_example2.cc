#include <stdio.h>
#include <stdlib.h>

#include "armadillo_socp.h"

#include <boost/math/special_functions/erf.hpp>

double CInv(double eps) {
  return sqrt(2.0) * boost::math::erf_inv(2 * eps - 1);
}

int main(int argc, char **argv) {
  // min x_1 + x_2
  // s.t || x || <= R
  //     x_1 > -0.5

  double R = 1.0;

  double eps = 0.95;
  double CInvEps = CInv(eps);

  Col<double> fCol = "-1";
  vector<Mat<double> > AMats(3);
  vector<Col<double> > BVecs(3);
  vector<Col<double> > CVecs(3);
  vector<Col<double> > DVecs(3);

  AMats[0] = "0.07";
  BVecs[0] = "0";
  //AMats[0].reset();
  //BVecs[0].reset();
  CVecs[0] = "0.608";
  DVecs[0] = "-0.4256";

  AMats[1].reset();
  BVecs[1].reset();
  CVecs[1] = "1";
  DVecs[1] = "0.0";

  AMats[2].reset();
  BVecs[2].reset();
  CVecs[2] = "-1";
  DVecs[2] = "1.0";

  Col<double> xCol;  // = "0 0";

  vector<Col<double> > ZVecs;
  vector<double> W;

  bool debug_socp = false;

  int ret;
  double foo = 0.5;
  while (/*ret != 0 && */ foo > 0.01) {
    AMats[0](0) = foo;
    printf("---------------------------------------------\n");
    printf("Checking for solution with variance = %2.2f\n", foo);
    printf("---------------------------------------------\n");
    xCol.reset();
    ZVecs.clear();
    W.clear();
    vector<double> u(fCol.n_elem, 1.0);
    vector<double> l(fCol.n_elem, -1e-9);
    ret = SolveSOCP(fCol, AMats, BVecs, CVecs, DVecs, xCol, ZVecs, W, u, l,
                    1e-6, debug_socp);
    foo -= 0.01;
    if (ret == 0) {
      xCol.print("x*");
      double tmp = -DVecs[0](0) / (CVecs[0](0) - AMats[0](0));
      printf("x >= %f\n", tmp);
    } else {
      fprintf(stderr, "Unable to find solution\n");
    }
    printf("==============================================\n");
  }

  AMats[0](0) = 0.0;
  printf("---------------------------------------------\n");
  printf("Checking for solution with variance = %2.2f\n", foo);
  printf("---------------------------------------------\n");
  xCol.reset();
  ZVecs.clear();
  W.clear();
  vector<double> u(fCol.n_elem, 1.0);
  vector<double> l(fCol.n_elem, -1e-9);
  ret = SolveSOCP(fCol, AMats, BVecs, CVecs, DVecs, xCol, ZVecs, W, u, l, 1e-6,
                  debug_socp);
  if (ret == 0) {
    xCol.print("x*");
    double tmp = -DVecs[0](0) / (CVecs[0](0) - AMats[0](0));
    printf("x >= %f\n", tmp);
  } else {
    fprintf(stderr, "Unable to find solution\n");
  }
  printf("==============================================\n");
}
