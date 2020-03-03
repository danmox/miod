#include <stdio.h>
#include <stdlib.h>

#include "socp.h"

int main(int argc, char **argv)
{
  // min x_1 + x_2
  // s.t || x || <= R
  //     x_1 > -0.5

  double R = 1.0;

  // Number of conic constraints
  int L = 2;

  // Number of rows per each constraint + 1
  int N[2] = {3, 1};

  // Number of variables
  int n = 2;
  
  // The linear objective functions
  double f[2] = {1, 1};

  // The cone constraints

  // A from \|Ax + b\|_2 \leq c'x + d
  // c from \|Ax + b\|_2 \leq c'x + d
  // A and c are stacked, e.g.
  // A = [A_1; c_1'; ... ; A_l; c_l'];
  // Note that A is defined column centric
  double A[8] ={1, 0, 0, 1,
		0, 1, 0, 0};

  // b from \|Ax + b\|_2 \leq c'x + d
  // d from \|Ax + b\|_2 \leq c'x + d
  // b and d are stacked, e.g.
  // b = [b_1; d_1; ... ; b_l; d_l];
  double b[4] = {0, 0, R, 0.5};

  // It is a true statement that (0, 0) is a feasible point
  double x[2] = {0, 0};
  
  // Compute the an initial strictly feasible dual point
  // It is possible to select a point that is always feasible and well posed
  // While z is analyically correct, w must be chosen such that \|z_i\| << w_i
  // z and w are stacked, e.g.
  // z = [z_1; w_1; ... ; z_l, w_l]
  double z[4] = {0.8, 1, 2, 0.2};

  double abs_tol = 1e-6;
  double rel_tol = 1e-4;
  double target = 0;
  int iter = 100;
  double Nu = 10;
  int info;
  int out_mode = 0;

  int mhist, nhist;
  int ndbl, nint;
  socp_getwork(L, N, n, iter, out_mode,
               &mhist, &nhist, &ndbl, &nint);

  int intwork[nint];
  double dblwork[ndbl];
  double hist[mhist*nhist];

  // SOCP optimization routine
  puts("Calling SOCP with:");
  printf("L: %i\n", L);
  printf("N: ");
  int foo;
  for (foo = 0; foo < L; foo++)
    printf("%i ", N[foo]);
  puts("");
  printf("n: %i\n", n);
  printf("f: ");
  for (foo = 0; foo < n; foo++)
    printf("%f ", f[foo]);
  puts("");
  puts("A:");
  int bar;
  int m = 0;
  for (foo = 0; foo < L; foo++)
    m += N[foo];
  for (bar = 0; bar < m; bar++)
    {
      for (foo = 0; foo < n; foo++)
        printf("\t%1.f", A[bar + m*foo]);
      printf("\n");
    }
  puts("b:");
  int baz = 0;
  for (foo = 0; foo < L; foo++)
    {
      for (bar = 0; bar < N[foo]; bar++, baz++)
        printf("%f ", b[baz]);
      printf("\n");
    }
  printf("x: ");
  for (foo = 0; foo < n; foo++)
    printf("%f ", x[foo]);
  puts("");
  puts("z:");
  baz = 0;
  for (foo = 0; foo < L; foo++)
    {
      for (bar = 0; bar < N[foo]; bar++, baz++)
        printf("%f ", z[baz]);
      printf("\n");
    }
  puts("");
  printf("abs_tol: %f\n", abs_tol);
  printf("rel_tol: %f\n", rel_tol);
  printf("target: %i\n", target);
  printf("iter: %i\n", iter);
  printf("Nu: %f\n", Nu);

  printf("out_mode: %i\n", out_mode);

  int err = socp(L, N, n, f, A, b, x, z,
                 abs_tol, rel_tol, target, &iter,
                 Nu, &info, out_mode, hist,
                 dblwork, intwork);
  
  if (err)
    {
      puts("Error in SOCP solve!");
      return -1;
    }

  printf("-------------------------\n");
  printf("Result from socp( )\n");

  printf("iter: %i\n", iter);
  printf("info: %i\n", info);

  printf("soln: ");
  for(foo = 0; foo < n; ++foo) {
    printf("%f ", x[foo]);
  }
  printf("\n");
}
