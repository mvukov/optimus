#ifndef THIRD_PARTY_BLAS_F77_H_
#define THIRD_PARTY_BLAS_F77_H_

extern "C" {

/* Level 1 BLAS */

extern double dasum_(const int* n, const double* dx, const int* incx);
extern void daxpy_(const int* n, const double* alpha, const double* dx,
                   const int* incx, double* dy, const int* incy);
extern void dcopy_(const int* n, const double* dx, const int* incx, double* dy,
                   const int* incy);
extern double ddot_(const int* n, const double* dx, const int* incx,
                    const double* dy, const int* incy);
extern double dnrm2_(const int* n, const double* dx, const int* incx);
extern void drot_(const int* n, double* dx, const int* incx, double* dy,
                  const int* incy, const double* c, const double* s);
extern void drotg_(const double* a, const double* b, double* c, double* s);
extern void drotm_(const int* n, double* dx, const int* incx, double* dy,
                   const int* incy, const double* dparam);
extern void drotmg_(const double* dd1, const double* dd2, const double* dx1,
                    const double* dy1, double* param);
extern void dscal_(const int* n, const double* alpha, double* dx,
                   const int* incx);
extern void dswap_(const int* n, double* dx, const int* incx, double* dy,
                   const int* incy);
extern int idamax_(const int* n, const double* dx, const int* incx);

/* Level 2 BLAS */

extern void dgbmv_(const char* trans, const int* m, const int* n, const int* kl,
                   const int* ku, const double* alpha, const double* a,
                   const int* lda, const double* x, const int* incx,
                   const double* beta, double* y, const int* incy);
extern void dgemv_(const char* trans, const int* m, const int* n,
                   const double* alpha, const double* a, const int* lda,
                   const double* x, const int* incx, const double* beta,
                   double* y, const int* incy);
extern void dsbmv_(const char* uplo, const int* n, const int* k,
                   const double* alpha, const double* a, const int* lda,
                   const double* x, const int* incx, const double* beta,
                   double* y, const int* incy);
extern void dspmv_(const char* uplo, const int* n, const double* alpha,
                   const double* ap, const double* x, const int* incx,
                   const double* beta, double* y, const int* incy);
extern void dsymv_(const char* uplo, const int* n, const double* alpha,
                   const double* a, const int* lda, const double* x,
                   const int* incx, const double* beta, double* y,
                   const int* incy);
extern void dtbmv_(const char* uplo, const char* trans, const char* diag,
                   const int* n, const int* k, const double* a, const int* lda,
                   double* x, const int* incx);
extern void dtpmv_(const char* uplo, const char* trans, const char* diag,
                   const int* n, const double* ap, double* x, const int* incx);
extern void dtrmv_(const char* uplo, const char* trans, const char* diag,
                   const int* n, const double* a, const int* lda, double* x,
                   const int* incx);
extern void dtbsv_(const char* uplo, const char* trans, const char* diag,
                   const int* n, const int* k, const double* a, const int* lda,
                   double* x, const int* incx);
extern void dtpsv_(const char* uplo, const char* trans, const char* diag,
                   const int* n, const double* ap, double* x, const int* incx);
extern void dtrsv_(const char* uplo, const char* trans, const char* diag,
                   const int* n, const double* a, const int* lda, double* x,
                   const int* incx);
extern void dger_(const int* m, const int* n, const double* alpha, double* x,
                  const int* incx, double* y, const int* incy, double* a,
                  const int* lda);
extern void dsyr_(const char* uplo, const int* n, const double* alpha,
                  const double* x, const int* incx, double* a, const int* lda);
extern void dspr_(const char* uplo, const int* n, const double* alpha,
                  const double* x, const int* incx, double* ap);
extern void dsyr2_(const char* uplo, const int* n, const double* alpha,
                   const double* x, const int* incx, const double* y,
                   const int* incy, double* a, const int* lda);
extern void dspr2_(const char* uplo, const int* n, const double* alpha,
                   const double* x, const int* incx, const double* y,
                   const int* incy, double* ap);

/* Level 3 BLAS */

extern void dgemm_(const char* transa, const char* transb, const int* m,
                   const int* n, const int* k, const double* alpha,
                   const double* a, const int* lda, const double* b,
                   const int* ldb, const double* beta, double* c,
                   const int* ldc);
extern void dtrsm_(const char* side, const char* uplo, const char* transa,
                   const char* diag, const int* m, const int* n,
                   const double* alpha, const double* a, const int* lda,
                   double* b, const int* ldb);
extern void dtrmm_(const char* side, const char* uplo, const char* transa,
                   const char* diag, const int* m, const int* n,
                   const double* alpha, const double* a, const int* lda,
                   double* b, const int* ldb);
extern void dsymm_(const char* side, const char* uplo, const int* m,
                   const int* n, const double* alpha, const double* a,
                   const int* lda, const double* b, const int* ldb,
                   const double* beta, double* c, const int* ldc);
extern void dsyrk_(const char* uplo, const char* trans, const int* n,
                   const int* k, const double* alpha, const double* a,
                   const int* lda, const double* beta, double* c,
                   const int* ldc);
extern void dsyr2k_(const char* uplo, const char* trans, const int* n,
                    const int* k, const double* alpha, const double* a,
                    const int* lda, const double* b, const int* ldb,
                    const double* beta, double* c, const int* ldc);

}  // extern "C"

#endif  // THIRD_PARTY_BLAS_F77_H_
