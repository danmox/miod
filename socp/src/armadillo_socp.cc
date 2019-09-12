#include "armadillo_socp.h"

int SolveSOCP(const Col<double> &fCol,
              const vector<Mat<double> > &AMats,
              const vector<Col<double> > &BVecs,
              const vector<Col<double> > &CVecs,
              const vector<Col<double> > &DVecs,
              Col<double> &xCol,
              vector<Col<double> > &ZVecs,
              vector<double> &W,
              double primal_upper_bound,
              double primal_lower_bound,
              double rel_tol,
              bool debug) {

  int n = fCol.n_elem;

  if (primal_upper_bound != primal_lower_bound) {
    vector<double> u(n, primal_upper_bound);
    vector<double> l(n, primal_lower_bound);

    return SolveSOCP(fCol, AMats, BVecs, CVecs, DVecs, xCol, ZVecs, W, u, l,
                     rel_tol, debug);
  } else {
    vector<double> u(1, primal_upper_bound);
    vector<double> l;

    return SolveSOCP(fCol, AMats, BVecs, CVecs, DVecs, xCol, ZVecs, W, u, l,
                     rel_tol, debug);
  }

}

int SolveSOCP(const Col<double> &fCol, const vector<Mat<double> > &AMats,
              const vector<Col<double> > &BVecs,
              const vector<Col<double> > &CVecs,
              const vector<Col<double> > &DVecs, Col<double> &xCol,
              vector<Col<double> > &ZVecs, vector<double> &W,
              const vector<double> &primal_upper_bound,
              const vector<double> &primal_lower_bound, double rel_tol,
              bool debug) {
  Mat<double> Au;
  Col<double> Bu, Cu, Du;

  vector<Mat<double> > AuBox;
  vector<Col<double> > BuBox, CuBox, DuBox;

  // error check on the sizes of these input matrices/vectors
  if (AMats.size() != BVecs.size() || AMats.size() != CVecs.size() ||
      AMats.size() != DVecs.size()) {  //||
                                       //     fCol.n_elem != AMats[0].n_cols) {
    fprintf(stderr, "[SolveSOCP] Problem with input dimensions\n");
    return -1;
  }

  // Remove zero (or close to) A-matrices
  vector<Mat<double> > AMatFixed(AMats);
  vector<Col<double> > BVecFixed(BVecs);

  bool mat_zero;
  for (unsigned int k = 0; k < AMatFixed.size(); ++k) {
    mat_zero = true;
    for (unsigned int i = 0; i < AMatFixed[k].n_rows; ++i) {
      for (unsigned int j = 0; j < AMatFixed[k].n_cols; ++j) {
        if (AMatFixed[k](i, j) > 1e-4) mat_zero = false;
      }
    }

    if (mat_zero) {
      AMatFixed[k].reset();
      BVecFixed[k].reset();
    }
  }

  // Debug: print constraint matrices
  if (debug) {
    fCol.print("f");
    for (unsigned int i = 0; i < AMatFixed.size(); ++i) {
      printf("Constraint %d:\n", i);
      printf("--------------\n");
      AMatFixed[i].print("A:");
      BVecFixed[i].t().print("B:");
      CVecs[i].t().print("C:");
      DVecs[i].t().print("D:");
    }
  }

  // Number of variables
  int n = fCol.n_elem;

  // Number of conic constraints
  int L = AMatFixed.size();

  // counter for elements of A
  unsigned int row_count = 0;
  for (int l = 0; l < L; ++l) {
    row_count += AMatFixed[l].n_rows + 1;
  }

  // Compute the an initial strictly feasible dual point
  // It is possible to select a point that is always feasible and well posed
  // While z is analytically correct, w must be chosen such that \|z_i\| << w_i
  // z and w are stacked, e.g.
  // z = [z_1; w_1; ... ; z_l, w_l]
  double *z = NULL;

  Col<double> zL(n);
  Col<double> zI, wI;

  bool use_primal_norm_bound = false;
  bool use_primal_box_bound = false;
  bool use_primal_upper_lower_bound = false;

  if (ZVecs.size() == 0 || ZVecs[0].n_elem == 0) {
    if (primal_upper_bound.size() != fCol.n_elem) {
      use_primal_norm_bound = true;

      if (debug) printf("Finding initial dual-feasible point\n");
      // Find initial dual-feasible point by...

      // (1) add an extra upper bound constraint bounding the primal variables
      // extra (upper bound) constraint
      row_count += n + 1;

      // Upper bound is of the form || x || <= upper_bound
      // || Ax + b || <= c'x + d
      Au = eye<Mat<double> >(n, n);
      Bu.zeros(n);
      Cu.zeros(n);
      Du.ones(1);
      Du(0) = primal_upper_bound[0];

      zL = fCol;
      z = new double[row_count];
      ZVecs.resize(L + 1);
      W.resize(L + 1);
      // set the first L z_i arbitrarily and w_i > || z_i ||
      unsigned int k = 0;
      for (int i = 0; i < L; i++) {
        zI.ones(AMatFixed[i].n_rows);
        wI.ones(1);
        for (unsigned int j = 0; j < AMatFixed[i].n_rows; ++j) {
          z[k++] = 1.0;  // means norm z_i = sqrt( AMatFixed[i].n_rows )
          zI(j) = 1.0;
        }
        z[k++] = sqrt(AMatFixed[i].n_rows) + 1.0;  // so w_i > || z_i ||
        wI(0) = sqrt(AMatFixed[i].n_rows) + 1.0;

        Col<double> ci = CVecs[i];

        if (AMatFixed[i].n_rows > 0) zL = zL - trans(AMatFixed[i]) * zI;
        if (ci.n_elem > 0) zL = zL - ci * wI(0);

        ZVecs[i] = zI;
        W[i] = wI(0);
        ZVecs[L] = zL;
      }
      // Now set z[L] s.t. sum_{i=1..L-1} (A_i'z_i + c_i w_i) + z[L] = f
      // i.e. z[L] = f - sum_{i=1..L-1} (A_i'z_i + c_i w_i)
      for (int i = 0; i < n; ++i) {
        z[k++] = zL(i);
      }

      z[k++] = norm(zL, 2) + 1.0;  // w_L > || z_L ||
      W[L] = norm(zL, 2) + 1.0;

      if (debug) {
        printf("Constraint %d:\n", L);
        printf("--------------\n");
        Au.print("A:");
        Bu.t().print("B:");
        Cu.t().print("C:");
        Du.t().print("D:");
      }
    } else {
      use_primal_box_bound = true;
      if (primal_lower_bound.size() == primal_upper_bound.size()) {
        use_primal_upper_lower_bound = true;
      }

      if (debug) {
        if (use_primal_upper_lower_bound)
          printf("Finding initial dual-feasible point with explicit "
                 "upper/lower bounds\n");
        else
          printf("Finding initial dual-feasible point with box norm\n");
      }

      AuBox.resize(2 * n);
      BuBox.resize(2 * n);
      CuBox.resize(2 * n);
      DuBox.resize(2 * n);

      // Find initial dual-feasible point by...

      // Upper bound
      for (int i = 0; i < n; ++i) {
        // (1) for each component of x,
        // add an extra upper bound constraint bounding the primal variables

        row_count += 1;

        // Upper bound is of the form x_i <= upper_bound
        // 0 <= c'x + d
        AuBox[i].reset();
        BuBox[i].reset();

        CuBox[i].zeros(n);
        CuBox[i](i) = -1;

        DuBox[i].ones(1);
        DuBox[i](0) = primal_upper_bound[i];
      }

      // Lower bound
      for (int i = n; i < 2 * n; ++i) {
        // (1) for each component of x,
        // add an extra lower bound constraint bounding the primal variables

        row_count += 1;

        // Lower bound is of the form x_i >= lower_bound
        // 0 <= c'x + d
        AuBox[i].reset();
        BuBox[i].reset();

        CuBox[i].zeros(n);
        CuBox[i](i - n) = 1;

        DuBox[i].ones(1);

        if (use_primal_upper_lower_bound) {
          DuBox[i](0) = -primal_lower_bound[i - n];
        } else {
          // Box norm is
          // x_i >= -upper_bound
          DuBox[i](0) = primal_upper_bound[i - n];
        }
      }

      zL = fCol;

      z = new double[row_count];
      ZVecs.resize(L + 2 * n);
      W.resize(L + 2 * n);
      // set the first L z_i arbitrarily and w_i > || z_i ||
      unsigned int k = 0;
      for (int i = 0; i < L; i++) {
        zI.ones(AMatFixed[i].n_rows);
        wI.ones(1);
        for (unsigned int j = 0; j < AMatFixed[i].n_rows; ++j) {
          z[k++] = 1.0;  // means norm z_i = sqrt( AMatFixed[i].n_rows )
          zI(j) = 1.0;
        }
        wI(0) = sqrt(AMatFixed[i].n_rows) + 1.0;  // so w_i > || z_i ||
        z[k++] = wI(0);

        Col<double> ci = CVecs[i];

        if (AMatFixed[i].n_rows > 0) zL = zL - trans(AMatFixed[i]) * zI;
        if (ci.n_elem > 0) zL = zL - ci * wI(0);

        ZVecs[i] = zI;
        W[i] = wI(0);
      }

      // Pick w_i to satisfy sum_{i=1..L-1} (A_i'z_i + c_i w_i) + sum_{i=L..L+n}
      // c_i w_i = f
      // where we know each c_i for an upper bound is zero except for a -1 in
      // the ith row
      // and each c_i for a lower bound is zero except for 1 in the ith row so
      // that
      // w[L+n+i] - w[L+i] = f - sum_{i=1..L-1} (A_i'z_i + c_i w_i)
      for (int i = 0; i < n; ++i) {
        W[L + n + i] = -1;
        W[L + i] = 1.0;

        while (W[L + n + i] < 0.1) {
          W[L + i] += 0.1;
          W[L + n + i] = zL(i) + W[L + i];
        }

        // vector z_{L+i} is null since we have no A_{L+i}
        ZVecs[L + i].reset();

        // vector z_{L+n+i} is null since we have no A_{L+n+i}
        ZVecs[L + n + i].reset();
      }

      for (int i = 0; i < 2 * n; ++i) {
        z[k++] = W[L + i];
      }

      if (debug) {
        for (unsigned int i = 0; i < AuBox.size(); ++i) {
          printf("Constraint %d:\n", L + i);
          printf("--------------\n");
          AuBox[i].print("A:");
          BuBox[i].t().print("B:");
          CuBox[i].t().print("C:");
          DuBox[i].t().print("D:");
        }
      }

    }

  } else {
    // Otherwise, use values supplied to this function
    z = new double[row_count];
    unsigned int k = 0;
    for (unsigned int i = 0; i < ZVecs.size(); ++i) {
      for (unsigned int j = 0; j < ZVecs[i].n_elem; ++j) {
        z[k++] = ZVecs[i](j);
      }
      z[k++] = W[i];
    }
  }

  if (debug) {
    printf("Dual variables\n");
    printf("--------------\n");
    for (int i = 0; i < ZVecs.size(); ++i) {
      printf("i=%d\n", i);
      ZVecs[i].print("z_i");
      printf("w_i=%2.2f\n", W[i]);
    }
  }

  ///////////////////////////////////////
  // Check if dual point is dual-feasible
  ///////////////////////////////////////
  if (debug) {
    Mat<double> check = fCol;
    check.zeros();

    for (int i = 0; i < AMatFixed.size(); ++i) {
      if (ZVecs[i].n_elem > 0) check += trans(AMatFixed[i]) * ZVecs[i];
      if (CVecs[i].n_elem > 0) check += W[i] * CVecs[i];
    }

    if (use_primal_norm_bound) {
      Mat<double> Atmp =
          eye<Mat<double> >(ZVecs.back().n_rows, ZVecs.back().n_rows);
      check += Atmp * ZVecs.back();
    } else {  // if(use_primal_box_bound)
      for (int i = 0; i < AuBox.size(); ++i) {
        check += W[L + i] * CuBox[i];
      }
    }

    check.print("Dual check: ");
  }

  ///////////////////////////////////////
  // End Check
  ///////////////////////////////////////

  // Initial feasible point
  double *x = new double[n];

  if (xCol.n_elem == 0) {
    // Find initial feasible point by formulating a 'Phase 1' problem
    Col<double> fNew = zeros<Col<double> >(n + 1);
    fNew(n) = 1;
    vector<Mat<double> > AMatNew(AMatFixed);
    vector<Col<double> > BVecNew(BVecFixed);
    vector<Col<double> > CVecNew(CVecs);
    vector<Col<double> > DVecNew(DVecs);

    // Resize constraint matrices and reform so that
    // || Ax + b || <= c'x + d
    // becomes
    // || Ax + b || <= c'x + d + alpha
    // where alpha is x_{n+1}
    for (unsigned int i = 0; i < AMatNew.size(); ++i) {

      //if(AMatNew[i].n_rows > 0)
      //AMatNew[i].set_size(n+1, n+1);

      AMatNew[i].set_size(AMatNew[i].n_rows, n + 1);

      CVecNew[i].set_size(n + 1);

      AMatNew[i].zeros();
      CVecNew[i].zeros();

      // Copy old AMat/CVec afer we resize
      for (unsigned int ii = 0; ii < AMatFixed[i].n_rows; ++ii)
        for (unsigned int jj = 0; jj < AMatFixed[i].n_cols; ++jj)
          AMatNew[i](ii, jj) = AMatFixed[i](ii, jj);

      for (unsigned int ii = 0; ii < CVecs[i].n_elem; ++ii)
        CVecNew[i](ii) = CVecs[i](ii);

      CVecNew[i](n) = 1;
    }

    // Now we solve the optimization problem
    // min alpha
    // s.t. || A_i x + b_i || <= c_i'x + d_i + alpha
    //
    // if alpha < 0, problem is feasible with the associated x
    // if alpha >= 0, problem is infeasible (that is, there is no strictly
    // feasible solution)

    // It is trivial to construct a feasible primal point by choosing alpha
    // large enough
    Col<double> xNew = zeros<Col<double> >(n + 1);
    // todo: figure out what gains are here in terms of finding points 'closer'
    // to feasible
    double alpha = 0;
    double delta = 0.25;
    // For each constraint,
    for (unsigned int i = 0; i < AMatNew.size(); ++i) {
      // compute alpha necessary for constraint satisfaction
      double tmp = -1.0 * dot(CVecNew[i], xNew) - DVecNew[i](0);
      if (AMatNew[i].n_rows > 0) {
        tmp += norm(AMatNew[i] * xNew + BVecNew[i], 2);
      }
      tmp += delta;
      if (tmp > alpha) {
        alpha = tmp;
      }
    }
    if (alpha > 1.0) {
      alpha = 0.999999999;
    }
    xNew(n) = alpha;
    // printf("using alpha %f\n", alpha);

    // Will find dual-feasible point automatically
    vector<Col<double> > ZVecNew;
    vector<double> WNew;

    if (debug)
      printf("Recursively calling SolveSOCP with new, feasibility problem\n");

    vector<double> u(primal_upper_bound);
    vector<double> l(primal_lower_bound);

    // If we have explicit upper/lower bounds (i.e. not norm bounds),
    // increment these bounds for the new expanded problem.
    if (u.size() == fCol.n_elem) {
      // Upper bound on 't'
      u.push_back(1.0);
    }
    if (l.size() == fCol.n_elem) {
      // Lower bound on 't'
      // make sure it is less than zero (otherwise program will not terminate)
      l.push_back(-1.0);
    }

    // Recursively call SolveSOCP with this new problem
    int ret = SolveSOCP(fNew, AMatNew, BVecNew, CVecNew, DVecNew, xNew, ZVecNew,
                        WNew, u, l, -1.0, debug);

    if (ret == 0 && xNew(n) < 0) {
      for (int i = 0; i < n; ++i)
        x[i] = xNew(i);
    } else {
      // printf("Constraints are infeasible (%f)\n", xNew(n));
      delete[] z;
      delete[] x;
      return -1;
    }
  } else {
    // Otherwise, use the values supplied to this function
    for (int i = 0; i < n; ++i) {
      x[i] = xCol(i);
    }
  }

  // number of rows per constraint + 1
  int *N = new int[L + (use_primal_norm_bound ? 1 : 0) +
                   (use_primal_box_bound ? 2 * n : 0)];
  for (int l = 0; l < L; ++l) {
    N[l] = AMatFixed[l].n_rows + 1;
  }
  if (use_primal_norm_bound) {
    N[L] = Au.n_rows + 1;
  }
  if (use_primal_box_bound) {
    for (int i = 0; i < 2 * n; ++i)
      N[L + i] = AuBox[i].n_rows + 1;
  }

  // Linear objective function
  double *f = new double[n];
  for (int i = 0; i < n; ++i)
    f[i] = fCol(i);

  // Cone constraints

  // A from \|Ax + b\|_2 \leq c'x + d
  // c from \|Ax + b\|_2 \leq c'x + d
  // A and c are stacked, e.g.
  // A = [A_1; c_1'; ... ; A_l; c_l'];
  // Note that A is defined column centric
  double *A = new double[row_count * n];
  unsigned int shift = 0;
  // For each constraint
  for (int l = 0; l < L; ++l) {
    // For each column (i.e. optmization variable)
    for (int col_idx = 0; col_idx < n; ++col_idx) {
      // For each row of the A_i matrix
      for (unsigned int i = 0; i < AMatFixed[l].n_rows; ++i) {
        A[i + col_idx * row_count + shift] = AMatFixed[l](i, col_idx);
      }
      // For the single row of c_i'
      A[AMatFixed[l].n_rows + col_idx * row_count + shift] = CVecs[l](col_idx);
    }

    shift += N[l];
  }

  if (use_primal_norm_bound) {
    // For each column (i.e. optmization variable)
    for (int col_idx = 0; col_idx < n; ++col_idx) {
      // For each row of the A_L matrix
      for (unsigned int i = 0; i < Au.n_rows; ++i) {
        A[i + col_idx * row_count + shift] = Au(i, col_idx);
      }
      // For the single row of c_i'
      A[Au.n_rows + col_idx * row_count + shift] = Cu(col_idx);
    }
  }

  if (use_primal_box_bound) {
    for (int k = 0; k < 2 * n; ++k) {
      // For each column (i.e. optmization variable)
      for (int col_idx = 0; col_idx < n; ++col_idx) {
        // For each row of the A_{L+k} matrix (should be zero)
        for (unsigned int i = 0; i < AuBox[k].n_rows; ++i) {
          A[i + col_idx * row_count + shift] = AuBox[k](i, col_idx);
        }
        // For the single row of c_i'
        A[AuBox[k].n_rows + col_idx * row_count + shift] = CuBox[k](col_idx);
      }
      shift += N[L + k];
    }
  }

  // check that the AC matrix is full rank
  Mat<double> AC(row_count, n);
  for (int col = 0; col < n; ++col) {
    for (int row = 0; row < row_count; ++row) {
      AC(row, col) = A[col * row_count + row];
    }
  }

  int rank = arma::rank(AC);

  if (debug)
    printf("\n AC of dim %d x %d has rank %d \n", AC.n_rows, AC.n_cols, rank);

  // b from \|Ax + b\|_2 \leq c'x + d
  // d from \|Ax + b\|_2 \leq c'x + d
  // b and d are stacked, e.g.
  // b = [b_1; d_1; ... ; b_l; d_l];
  double *b = new double[row_count];
  shift = 0;
  // For each constraint
  for (int l = 0; l < L; ++l) {
    // For each row of the b_i vector
    for (unsigned int i = 0; i < BVecFixed[l].n_elem; ++i) {
      b[i + shift] = BVecFixed[l](i);
    }
    // For the single entry of d_i
    b[BVecFixed[l].n_elem + shift] = DVecs[l](0);

    shift += N[l];
  }

  if (use_primal_norm_bound) {
    // For each row of the b_i vector
    for (unsigned int i = 0; i < Bu.n_elem; ++i) {
      b[i + shift] = Bu(i);
    }
    // For the single entry of d_i
    b[Bu.n_elem + shift] = Du(0);
  }

  if (use_primal_box_bound) {
    for (int k = 0; k < 2 * n; ++k) {
      // For each row of the b_{L+k} vector
      for (unsigned int i = 0; i < BuBox[k].n_elem; ++i) {
        b[i + shift] = BuBox[k](i);
      }
      // For the single entry of d_i
      b[BuBox[k].n_elem + shift] = DuBox[k](0);

      shift += N[L + k];
    }
  }

  double abs_tol = 1e-6;
  double target = 0;
  int iter = 100;
  double Nu = 10;
  int info;
  int out_mode = 0;

  int mhist, nhist;
  int ndbl, nint;
  socp_getwork(
      L + (use_primal_norm_bound ? 1 : 0) + (use_primal_box_bound ? 2 * n : 0),
      N, n, iter, out_mode, &mhist, &nhist, &ndbl, &nint);

  int intwork[nint];
  double dblwork[ndbl];
  double hist[mhist * nhist];

  if (debug) {
    // SOCP optimization routine
    puts("Calling SOCP with:");
    printf("L: %i\n", L + (use_primal_norm_bound ? 1 : 0) +
                          (use_primal_box_bound ? 2 * n : 0));
    printf("N: ");
    for (int foo = 0; foo < L + (use_primal_norm_bound ? 1 : 0) +
                                (use_primal_box_bound ? 2 * n : 0);
         foo++)
      printf("%i ", N[foo]);
    puts("");
    printf("n: %i\n", n);
    printf("f: ");
    for (int foo = 0; foo < n; foo++)
      printf("%f ", f[foo]);
    puts("");
    puts("A:");
    int bar;
    int m = 0;
    for (int foo = 0; foo < L + (use_primal_norm_bound ? 1 : 0) +
                                (use_primal_box_bound ? 2 * n : 0);
         foo++)
      m += N[foo];
    for (bar = 0; bar < m; bar++) {
      for (int foo = 0; foo < n; foo++)
        printf("\t%2.2f", A[bar + m * foo]);
      printf("\n");
    }
    puts("b:");
    int baz = 0;
    for (int foo = 0; foo < L + (use_primal_norm_bound ? 1 : 0) +
                                (use_primal_box_bound ? 2 * n : 0);
         foo++) {
      for (bar = 0; bar < N[foo]; bar++, baz++)
        printf("%f ", b[baz]);
      printf("\n");
    }
    printf("x: ");
    for (int foo = 0; foo < n; foo++)
      printf("%f ", x[foo]);
    puts("");
    puts("z:");
    baz = 0;
    for (int foo = 0; foo < L + (use_primal_norm_bound ? 1 : 0) +
                                (use_primal_box_bound ? 2 * n : 0);
         foo++) {
      for (bar = 0; bar < N[foo]; bar++, baz++)
        printf("%f ", z[baz]);
      printf("\n");
    }
    puts("");
    printf("abs_tol: %f\n", abs_tol);
    printf("rel_tol: %f\n", rel_tol);
    printf("target: %f\n", target);
    printf("iter: %i\n", iter);
    printf("Nu: %f\n", Nu);

    printf("out_mode: %i\n", out_mode);

  }

  int err = socp(
      L + (use_primal_norm_bound ? 1 : 0) + (use_primal_box_bound ? 2 * n : 0),
      N, n, f, A, b, x, z, abs_tol, rel_tol, target, &iter, Nu, &info, out_mode,
      hist, dblwork, intwork);

  if (err) {
    puts("Error in SOCP solve!");
    delete[] z;
    delete[] x;
    delete[] N;
    delete[] f;
    delete[] b;
    delete[] A;

    return -1;
  }

  if (debug) {
    printf("-------------------------\n");
    printf("Result from socp( )\n");

    printf("iter: %i\n", iter);
    printf("info: %i\n", info);

    printf("soln: ");
    for (int foo = 0; foo < n; ++foo) {
      printf("%f ", x[foo]);
    }
    printf("f(x_soln): ");
    double tmp = 0;
    for (int foo = 0; foo < n; ++foo) {
      tmp += f[foo] * x[foo];
    }
    printf("%2.2f\n", tmp);

    if (out_mode > 0) {
      printf("hist: \n");
      for (int bar = 0; bar < mhist; bar++) {
        for (int foo = 0; foo < nhist; foo++)
          printf("\t%2.2f", hist[bar + mhist * foo]);
        printf("\n");
      }
    }
  }
  // Put solution back in xCol
  if ((int)(xCol.n_elem) != n) xCol.ones(n);

  for (int i = 0; i < n; ++i) {
    xCol(i) = x[i];
  }

  delete[] z;
  delete[] x;
  delete[] N;
  delete[] f;
  delete[] b;
  delete[] A;

  return 0;

}
