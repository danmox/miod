#include <stdio.h>
#include <stdlib.h>

#include "armadillo_socp.h"

int main(int argc, char **argv) {
  // min x_1 + x_2
  // s.t || x || <= R
  //     x_1 > -0.5

  double R = 2.0;

  Col<double> fCol = "-1 -1";
  vector<Mat<double> > AMats(2);
  vector<Col<double> > BVecs(2);
  vector<Col<double> > CVecs(2);
  vector<Col<double> > DVecs(2);

  AMats[0] = "1 0; 0 1";
  BVecs[0] = "0 0";
  CVecs[0] = "0 0";
  DVecs[0] = "0";
  DVecs[0](0) = R;

  AMats[1].reset();
  BVecs[1].reset();
  CVecs[1] = "1 0";
  DVecs[1] = "0.5";

  //while (1) {

    Col<double> xCol;  // = "0 0";
    vector<Col<double> > ZVecs;
    vector<double> W;

    vector<double> u(2, 1.0);
    vector<double> l;  //(2, -1.0);

    int ret = SolveSOCP(fCol, AMats, BVecs, CVecs, DVecs, xCol, ZVecs, W, u, l,
                        1e-4, true);
    //int ret = SolveSOCP(fCol, AMats, BVecs, CVecs, DVecs, xCol, ZVecs, W, 1.0,
    //1.0, 1e-4, true);
    if (ret == 0) {
      xCol.print("x*");
    } else {
      fprintf(stderr, "Unable to find solution\n");
    }
    //}
}
