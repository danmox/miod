#include <stdio.h>
#include <stdlib.h>

#include "armadillo_socp.h"

int main(int argc, char **argv) {
  // min x_1 + x_2
  // s.t || x || <= R
  //     x_1 > -0.5

  double R = 1.0;

  Col<double> fCol = "0 -1";
  vector<Mat<double> > AMats(5);
  vector<Col<double> > BVecs(5);
  vector<Col<double> > CVecs(5);
  vector<Col<double> > DVecs(5);

  AMats[0] = "0.0 0.0; 0.0 0.1";
  BVecs[0] = "0 0";
  //AMats[0].reset();
  //BVecs[0].reset();
  CVecs[0] = "0 0.608";
  DVecs[0] = "-0.4256";

  AMats[1].reset();
  BVecs[1].reset();
  CVecs[1] = "1 0";
  DVecs[1] = "0.0";

  AMats[2].reset();
  BVecs[2].reset();
  CVecs[2] = "-1 0";
  DVecs[2] = "1.0";

  AMats[3].reset();
  BVecs[3].reset();
  CVecs[3] = "0 1";
  DVecs[3] = "0.0";

  AMats[4].reset();
  BVecs[4].reset();
  CVecs[4] = "0 -1";
  DVecs[4] = "1.0";

  Col<double> xCol;  // = "0 0";

  vector<Col<double> > ZVecs;
  vector<double> W;

  vector<double> u(fCol.n_elem, 1.0);
  vector<double> l(fCol.n_elem, -1e-9);

  int ret = SolveSOCP(fCol, AMats, BVecs, CVecs, DVecs, xCol, ZVecs, W, u, l,
                      1e-4, true);

  //int ret = SolveSOCP(fCol, AMats, BVecs, CVecs, DVecs, xCol, ZVecs, W, 1.0,
  //1.0, 1e-4, true);

  if (ret == 0) {
    xCol.print("x*");
  } else {
    fprintf(stderr, "Unable to find solution\n");
  }

}
