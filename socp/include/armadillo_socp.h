#ifndef _ARMADILLO_SOCP_H_
#define _ARMADILLO_SOCP_H_

#include "armadillo"
using namespace arma;

extern "C" {
#include "socp.h"
}

#include <vector>
using namespace std;

int SolveSOCP(const Col<double> &fCol,
              const vector<Mat<double> > &AMats,
              const vector<Col<double> > &BVecs,
              const vector<Col<double> > &CVecs,
              const vector<Col<double> > &DVecs,
              Col<double> &xCol,
              vector<Col<double> > &ZVecs,
              vector<double> &W,
              double primal_upper_bound = 1.0,
              double primal_lower_bound = 1.0,
              double rel_tol = 1e-4,
              bool debug = false);

int SolveSOCP(const Col<double> &fCol,
              const vector<Mat<double> > &AMats,
              const vector<Col<double> > &BVecs,
              const vector<Col<double> > &CVecs,
              const vector<Col<double> > &DVecs,
              Col<double> &xCol,
              vector<Col<double> > &ZVecs,
              vector<double> &W,
              const vector<double> &primal_upper_bound,
              const vector<double> &primal_lower_bound,
              double rel_tol = 1e-4,
              bool debug = false);
#endif
