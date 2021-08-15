#ifndef THIRD_PARTY_LAPACK_F77_H_
#define THIRD_PARTY_LAPACK_F77_H_

extern "C" {

extern void dbdsqr_(const char* uplo, const int* n, const int* ncvt,
                    const int* nru, const int* ncc, double* d, double* e,
                    double* vt, const int* ldvt, double* u, const int* ldu,
                    double* c, const int* ldc, double* work, int* info);
extern void ddisna_(const char* job, const int* m, const int* n, double* d,
                    double* sep, int* info);

extern void dgbbrd_(const char* vect, const int* m, const int* n,
                    const int* ncc, const int* kl, const int* ku, double* ab,
                    const int* ldab, double* d, double* e, double* q,
                    const int* ldq, double* pt, const int* ldpt, double* c,
                    const int* ldc, double* work, int* info);
extern void dgbcon_(const char* norm, const int* n, const int* kl,
                    const int* ku, double* ab, const int* ldab, int* ipiv,
                    const double* anorm, double* rcond, double* work,
                    int* iwork, int* info);
extern void dgbequ_(const int* m, const int* n, const int* kl, const int* ku,
                    double* ab, const int* ldab, double* r, double* c,
                    double* rowcnd, double* colcnd, double* amax, int* info);
extern void dgbrfs_(const char* trans, const int* n, const int* kl,
                    const int* ku, const int* nrhs, double* ab, const int* ldab,
                    double* afb, const int* ldafb, int* ipiv, double* b,
                    const int* ldb, double* x, const int* ldx, double* ferr,
                    double* berr, double* work, int* iwork, int* info);
extern void dgbsv_(const int* n, const int* kl, const int* ku, const int* nrhs,
                   double* ab, const int* ldab, int* ipiv, double* b,
                   const int* ldb, int* info);
extern void dgbsvx_(const int* fact, const char* trans, const int* n,
                    const int* kl, const int* ku, const int* nrhs, double* ab,
                    const int* ldab, double* afb, const int* ldafb, int* ipiv,
                    const char* equed, double* r, double* c, double* b,
                    const int* ldb, double* x, const int* ldx, double* rcond,
                    double* ferr, double* berr, double* work, int* iwork,
                    int* info);
extern void dgbtf2_(const int* m, const int* n, const int* kl, const int* ku,
                    double* ab, const int* ldab, int* ipiv, int* info);
extern void dgbtrf_(const int* m, const int* n, const int* kl, const int* ku,
                    double* ab, const int* ldab, int* ipiv, int* info);
extern void dgbtrs_(const char* trans, const int* n, const int* kl,
                    const int* ku, const int* nrhs, const double* ab,
                    const int* ldab, const int* ipiv, double* b, const int* ldb,
                    int* info);

extern void dgebak_(const char* job, const char* side, const int* n,
                    const int* ilo, const int* ihi, double* scale, const int* m,
                    double* v, const int* ldv, int* info);
extern void dgebal_(const char* job, const int* n, double* a, const int* lda,
                    int* ilo, int* ihi, double* scale, int* info);
extern void dgebd2_(const int* m, const int* n, double* a, const int* lda,
                    double* d, double* e, double* tauq, double* taup,
                    double* work, int* info);
extern void dgebrd_(const int* m, const int* n, double* a, const int* lda,
                    double* d, double* e, double* tauq, double* taup,
                    double* work, const int* lwork, int* info);
extern void dgecon_(const char* norm, const int* n, const double* a,
                    const int* lda, const double* anorm, double* rcond,
                    double* work, int* iwork, int* info);
extern void dgeequ_(const int* m, const int* n, double* a, const int* lda,
                    double* r, double* c, double* rowcnd, double* colcnd,
                    double* amax, int* info);
extern void dgees_(const char* jobvs, const char* sort,
                   int (*select)(const double*, const double*), const int* n,
                   double* a, const int* lda, int* sdim, double* wr, double* wi,
                   double* vs, const int* ldvs, double* work, const int* lwork,
                   int* bwork, int* info);
extern void dgeesx_(const char* jobvs, const char* sort,
                    int (*select)(const double*, const double*),
                    const char* sense, const int* n, double* a, const int* lda,
                    int* sdim, double* wr, double* wi, double* vs,
                    const int* ldvs, double* rconde, double* rcondv,
                    double* work, const int* lwork, int* iwork,
                    const int* liwork, int* bwork, int* info);
extern void dgeev_(const char* jobvl, const char* jobvr, const int* n,
                   double* a, const int* lda, double* wr, double* wi,
                   double* vl, const int* ldvl, double* vr, const int* ldvr,
                   double* work, const int* lwork, int* info);
extern void dgeevx_(const char* balanc, const char* jobvl, const char* jobvr,
                    const char* sense, const int* n, double* a, const int* lda,
                    double* wr, double* wi, double* vl, const int* ldvl,
                    double* vr, const int* ldvr, int* ilo, int* ihi,
                    double* scale, double* abnrm, double* rconde,
                    double* rcondv, double* work, const int* lwork, int* iwork,
                    int* info);
extern void dgegv_(const char* jobvl, const char* jobvr, const int* n,
                   double* a, const int* lda, double* b, const int* ldb,
                   double* alphar, double* alphai, const double* beta,
                   double* vl, const int* ldvl, double* vr, const int* ldvr,
                   double* work, const int* lwork, int* info);
extern void dgehd2_(const int* n, const int* ilo, const int* ihi, double* a,
                    const int* lda, double* tau, double* work, int* info);
extern void dgehrd_(const int* n, const int* ilo, const int* ihi, double* a,
                    const int* lda, double* tau, double* work, const int* lwork,
                    int* info);
extern void dgelq2_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, double* work, int* info);
extern void dgelqf_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, double* work, const int* lwork, int* info);
extern void dgels_(const char* trans, const int* m, const int* n,
                   const int* nrhs, double* a, const int* lda, double* b,
                   const int* ldb, double* work, const int* lwork, int* info);
extern void dgelss_(const int* m, const int* n, const int* nrhs, double* a,
                    const int* lda, double* b, const int* ldb, double* s,
                    double* rcond, int* rank, double* work, const int* lwork,
                    int* info);
extern void dgelsy_(const int* m, const int* n, const int* nrhs, double* a,
                    const int* lda, double* b, const int* ldb, int* jpvt,
                    const double* rcond, int* rank, double* work,
                    const int* lwork, int* info);
extern void dgeql2_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, double* work, int* info);
extern void dgeqlf_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, double* work, const int* lwork, int* info);
extern void dgeqp3_(const int* m, const int* n, double* a, const int* lda,
                    int* jpvt, double* tau, double* work, const int* lwork,
                    int* info);
extern void dgeqpf_(const int* m, const int* n, double* a, const int* lda,
                    int* jpvt, double* tau, double* work, int* info);
extern void dgeqr2_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, double* work, int* info);
extern void dgeqrf_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, double* work, const int* lwork, int* info);
extern void dgerfs_(const char* trans, const int* n, const int* nrhs, double* a,
                    const int* lda, double* af, const int* ldaf, int* ipiv,
                    double* b, const int* ldb, double* x, const int* ldx,
                    double* ferr, double* berr, double* work, int* iwork,
                    int* info);
extern void dgerq2_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, double* work, int* info);
extern void dgerqf_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, double* work, const int* lwork, int* info);
extern void dgesv_(const int* n, const int* nrhs, double* a, const int* lda,
                   int* ipiv, double* b, const int* ldb, int* info);
extern void dgesvd_(const char* jobu, const char* jobvt, const int* m,
                    const int* n, double* a, const int* lda, double* s,
                    double* u, const int* ldu, double* vt, const int* ldvt,
                    double* work, const int* lwork, int* info);
extern void dgesvx_(const int* fact, const char* trans, const int* n,
                    const int* nrhs, double* a, const int* lda, double* af,
                    const int* ldaf, int* ipiv, char* equed, double* r,
                    double* c, double* b, const int* ldb, double* x,
                    const int* ldx, double* rcond, double* ferr, double* berr,
                    double* work, int* iwork, int* info);
extern void dgetf2_(const int* m, const int* n, double* a, const int* lda,
                    int* ipiv, int* info);
extern void dgetrf_(const int* m, const int* n, double* a, const int* lda,
                    int* ipiv, int* info);
extern void dgetri_(const int* n, double* a, const int* lda, int* ipiv,
                    double* work, const int* lwork, int* info);
extern void dgetrs_(const char* trans, const int* n, const int* nrhs,
                    const double* a, const int* lda, const int* ipiv, double* b,
                    const int* ldb, int* info);

extern void dggbak_(const char* job, const char* side, const int* n,
                    const int* ilo, const int* ihi, double* lscale,
                    double* rscale, const int* m, double* v, const int* ldv,
                    int* info);
extern void dggbal_(const char* job, const int* n, double* a, const int* lda,
                    double* b, const int* ldb, int* ilo, int* ihi,
                    double* lscale, double* rscale, double* work, int* info);
extern void dgges_(const char* jobvsl, const char* jobvsr, const char* sort,
                   int (*delztg)(double*, double*, double*), const int* n,
                   double* a, const int* lda, double* b, const int* ldb,
                   double* alphar, double* alphai, const double* beta,
                   double* vsl, const int* ldvsl, double* vsr, const int* ldvsr,
                   double* work, const int* lwork, int* bwork, int* info);

extern void dggglm_(const int* n, const int* m, const int* p, double* a,
                    const int* lda, double* b, const int* ldb, double* d,
                    double* x, double* y, double* work, const int* lwork,
                    int* info);
extern void dgghrd_(const char* compq, const char* compz, const int* n,
                    const int* ilo, const int* ihi, double* a, const int* lda,
                    double* b, const int* ldb, double* q, const int* ldq,
                    double* z, const int* ldz, int* info);
extern void dgglse_(const int* m, const int* n, const int* p, double* a,
                    const int* lda, double* b, const int* ldb, double* c,
                    double* d, double* x, double* work, const int* lwork,
                    int* info);
extern void dggqrf_(const int* n, const int* m, const int* p, double* a,
                    const int* lda, double* taua, double* b, const int* ldb,
                    double* taub, double* work, const int* lwork, int* info);
extern void dggrqf_(const int* m, const int* p, const int* n, double* a,
                    const int* lda, double* taua, double* b, const int* ldb,
                    double* taub, double* work, const int* lwork, int* info);
extern void dggsvd_(const char* jobu, const char* jobv, const char* jobq,
                    const int* m, const int* n, const int* p, const int* k,
                    const int* l, double* a, const int* lda, double* b,
                    const int* ldb, const double* alpha, const double* beta,
                    double* u, const int* ldu, double* v, const int* ldv,
                    double* q, const int* ldq, double* work, int* iwork,
                    int* info);

extern void dgtcon_(const char* norm, const int* n, double* dl, double* d,
                    double* du, double* du2, int* ipiv, const double* anorm,
                    double* rcond, double* work, int* iwork, int* info);
extern void dgtrfs_(const char* trans, const int* n, const int* nrhs,
                    double* dl, double* d, double* du, double* dlf, double* df,
                    double* duf, double* du2, int* ipiv, double* b,
                    const int* ldb, double* x, const int* ldx, double* ferr,
                    double* berr, double* work, int* iwork, int* info);
extern void dgtsv_(const int* n, const int* nrhs, double* dl, double* d,
                   double* du, double* b, const int* ldb, int* info);
extern void dgtsvx_(const int* fact, const char* trans, const int* n,
                    const int* nrhs, double* dl, double* d, double* du,
                    double* dlf, double* df, double* duf, double* du2,
                    int* ipiv, double* b, const int* ldb, double* x,
                    const int* ldx, double* rcond, double* ferr, double* berr,
                    double* work, int* iwork, int* info);
extern void dgttrf_(const int* n, double* dl, double* d, double* du,
                    double* du2, int* ipiv, int* info);
extern void dgttrs_(const char* trans, const int* n, const int* nrhs,
                    double* dl, double* d, double* du, double* du2, int* ipiv,
                    double* b, const int* ldb, int* info);

extern void dopgtr_(const char* uplo, const int* n, const double* ap,
                    const double* tau, double* q, const int* ldq, double* work,
                    int* info);
extern void dopmtr_(const char* side, const char* uplo, const char* trans,
                    const int* m, const int* n, const double* ap,
                    const double* tau, double* c, const int* ldc, double* work,
                    int* info);
extern void dorg2l_(const int* m, const int* n, const int* k, double* a,
                    const int* lda, const double* tau, double* work, int* info);
extern void dorg2r_(const int* m, const int* n, const int* k, double* a,
                    const int* lda, const double* tau, double* work, int* info);
extern void dorgbr_(const char* vect, const int* m, const int* n, const int* k,
                    double* a, const int* lda, const double* tau, double* work,
                    const int* lwork, int* info);
extern void dorghr_(const int* n, const int* ilo, const int* ihi, double* a,
                    const int* lda, const double* tau, double* work,
                    const int* lwork, int* info);
extern void dorgl2_(const int* m, const int* n, const int* k, double* a,
                    const int* lda, const double* tau, double* work, int* info);
extern void dorglq_(const int* m, const int* n, const int* k, double* a,
                    const int* lda, const double* tau, double* work,
                    const int* lwork, int* info);
extern void dorgql_(const int* m, const int* n, const int* k, double* a,
                    const int* lda, const double* tau, double* work,
                    const int* lwork, int* info);
extern void dorgqr_(const int* m, const int* n, const int* k, double* a,
                    const int* lda, const double* tau, double* work,
                    const int* lwork, int* info);
extern void dorgr2_(const int* m, const int* n, const int* k, double* a,
                    const int* lda, const double* tau, double* work, int* info);
extern void dorgrq_(const int* m, const int* n, const int* k, double* a,
                    const int* lda, const double* tau, double* work,
                    const int* lwork, int* info);
extern void dorgtr_(const char* uplo, const int* n, double* a, const int* lda,
                    const double* tau, double* work, const int* lwork,
                    int* info);
extern void dorm2l_(const char* side, const char* trans, const int* m,
                    const int* n, const int* k, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    int* info);
extern void dorm2r_(const char* side, const char* trans, const int* m,
                    const int* n, const int* k, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    int* info);
extern void dormbr_(const char* vect, const char* side, const char* trans,
                    const int* m, const int* n, const int* k, const double* a,
                    const int* lda, const double* tau, double* c,
                    const int* ldc, double* work, const int* lwork, int* info);
extern void dormhr_(const char* side, const char* trans, const int* m,
                    const int* n, const int* ilo, const int* ihi,
                    const double* a, const int* lda, const double* tau,
                    double* c, const int* ldc, double* work, const int* lwork,
                    int* info);
extern void dorml2_(const char* side, const char* trans, const int* m,
                    const int* n, const int* k, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    int* info);
extern void dormlq_(const char* side, const char* trans, const int* m,
                    const int* n, const int* k, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    const int* lwork, int* info);
extern void dormql_(const char* side, const char* trans, const int* m,
                    const int* n, const int* k, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    const int* lwork, int* info);
extern void dormqr_(const char* side, const char* trans, const int* m,
                    const int* n, const int* k, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    const int* lwork, int* info);
extern void dormr2_(const char* side, const char* trans, const int* m,
                    const int* n, const int* k, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    int* info);
extern void dormrq_(const char* side, const char* trans, const int* m,
                    const int* n, const int* k, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    const int* lwork, int* info);
extern void dormtr_(const char* side, const char* uplo, const char* trans,
                    const int* m, const int* n, const double* a, const int* lda,
                    const double* tau, double* c, const int* ldc, double* work,
                    const int* lwork, int* info);

extern void dpbcon_(const char* uplo, const int* n, const int* kd,
                    const double* ab, const int* ldab, const double* anorm,
                    double* rcond, double* work, int* iwork, int* info);
extern void dpbequ_(const char* uplo, const int* n, const int* kd,
                    const double* ab, const int* ldab, double* s, double* scond,
                    double* amax, int* info);
extern void dpbrfs_(const char* uplo, const int* n, const int* kd,
                    const int* nrhs, const double* ab, const int* ldab,
                    const double* afb, const int* ldafb, const double* b,
                    const int* ldb, double* x, const int* ldx, double* ferr,
                    double* berr, double* work, int* iwork, int* info);
extern void dpbstf_(const char* uplo, const int* n, const int* kd, double* ab,
                    const int* ldab, int* info);
extern void dpbsv_(const char* uplo, const int* n, const int* kd,
                   const int* nrhs, double* ab, const int* ldab, double* b,
                   const int* ldb, int* info);
extern void dpbsvx_(const int* fact, const char* uplo, const int* n,
                    const int* kd, const int* nrhs, double* ab, const int* ldab,
                    double* afb, const int* ldafb, char* equed, double* s,
                    double* b, const int* ldb, double* x, const int* ldx,
                    double* rcond, double* ferr, double* berr, double* work,
                    int* iwork, int* info);
extern void dpbtf2_(const char* uplo, const int* n, const int* kd, double* ab,
                    const int* ldab, int* info);
extern void dpbtrf_(const char* uplo, const int* n, const int* kd, double* ab,
                    const int* ldab, int* info);
extern void dpbtrs_(const char* uplo, const int* n, const int* kd,
                    const int* nrhs, const double* ab, const int* ldab,
                    double* b, const int* ldb, int* info);

extern void dpocon_(const char* uplo, const int* n, const double* a,
                    const int* lda, const double* anorm, double* rcond,
                    double* work, int* iwork, int* info);
extern void dpoequ_(const int* n, const double* a, const int* lda, double* s,
                    double* scond, double* amax, int* info);
extern void dporfs_(const char* uplo, const int* n, const int* nrhs,
                    const double* a, const int* lda, const double* af,
                    const int* ldaf, const double* b, const int* ldb, double* x,
                    const int* ldx, double* ferr, double* berr, double* work,
                    int* iwork, int* info);
extern void dposv_(const char* uplo, const int* n, const int* nrhs, double* a,
                   const int* lda, double* b, const int* ldb, int* info);
extern void dposvx_(const int* fact, const char* uplo, const int* n,
                    const int* nrhs, double* a, const int* lda, double* af,
                    const int* ldaf, char* equed, double* s, double* b,
                    const int* ldb, double* x, const int* ldx, double* rcond,
                    double* ferr, double* berr, double* work, int* iwork,
                    int* info);
extern void dpotf2_(const char* uplo, const int* n, double* a, const int* lda,
                    int* info);
extern void dpotrf_(const char* uplo, const int* n, double* a, const int* lda,
                    int* info);
extern void dpotri_(const char* uplo, const int* n, double* a, const int* lda,
                    int* info);
extern void dpotrs_(const char* uplo, const int* n, const int* nrhs,
                    const double* a, const int* lda, double* b, const int* ldb,
                    int* info);
extern void dppcon_(const char* uplo, const int* n, const double* ap,
                    const double* anorm, double* rcond, double* work,
                    int* iwork, int* info);
extern void dppequ_(const char* uplo, const int* n, const double* ap, double* s,
                    double* scond, double* amax, int* info);

extern void dpprfs_(const char* uplo, const int* n, const int* nrhs,
                    const double* ap, const double* afp, const double* b,
                    const int* ldb, double* x, const int* ldx, double* ferr,
                    double* berr, double* work, int* iwork, int* info);
extern void dppsv_(const char* uplo, const int* n, const int* nrhs,
                   const double* ap, double* b, const int* ldb, int* info);
extern void dppsvx_(const int* fact, const char* uplo, const int* n,
                    const int* nrhs, double* ap, double* afp, char* equed,
                    double* s, double* b, const int* ldb, double* x,
                    const int* ldx, double* rcond, double* ferr, double* berr,
                    double* work, int* iwork, int* info);
extern void dpptrf_(const char* uplo, const int* n, double* ap, int* info);
extern void dpptri_(const char* uplo, const int* n, double* ap, int* info);
extern void dpptrs_(const char* uplo, const int* n, const int* nrhs,
                    const double* ap, double* b, const int* ldb, int* info);

extern void dptcon_(const int* n, const double* d, const double* e,
                    const double* anorm, double* rcond, double* work,
                    int* info);
extern void dpteqr_(const char* compz, const int* n, double* d, double* e,
                    double* z, const int* ldz, double* work, int* info);
extern void dptrfs_(const int* n, const int* nrhs, const double* d,
                    const double* e, const double* df, const double* ef,
                    const double* b, const int* ldb, double* x, const int* ldx,
                    double* ferr, double* berr, double* work, int* info);
extern void dptsv_(const int* n, const int* nrhs, double* d, double* e,
                   double* b, const int* ldb, int* info);
extern void dptsvx_(const int* fact, const int* n, const int* nrhs,
                    const double* d, const double* e, double* df, double* ef,
                    const double* b, const int* ldb, double* x, const int* ldx,
                    double* rcond, double* ferr, double* berr, double* work,
                    int* info);
extern void dpttrf_(const int* n, double* d, double* e, int* info);
extern void dpttrs_(const int* n, const int* nrhs, const double* d,
                    const double* e, double* b, const int* ldb, int* info);
extern void drscl_(const int* n, const double* da, double* x, const int* incx);

extern void dsbev_(const char* jobz, const char* uplo, const int* n,
                   const int* kd, double* ab, const int* ldab, double* w,
                   double* z, const int* ldz, double* work, int* info);
extern void dsbevd_(const char* jobz, const char* uplo, const int* n,
                    const int* kd, double* ab, const int* ldab, double* w,
                    double* z, const int* ldz, double* work, const int* lwork,
                    int* iwork, const int* liwork, int* info);
extern void dsbevx_(const char* jobz, const char* range, const char* uplo,
                    const int* n, const int* kd, double* ab, const int* ldab,
                    double* q, const int* ldq, const double* vl,
                    const double* vu, const int* il, const int* iu,
                    const double* abstol, int* m, double* w, double* z,
                    const int* ldz, double* work, int* iwork, int* ifail,
                    int* info);
extern void dsbgst_(const char* vect, const char* uplo, const int* n,
                    const int* ka, const int* kb, double* ab, const int* ldab,
                    double* bb, const int* ldbb, double* x, const int* ldx,
                    double* work, int* info);
extern void dsbgv_(const char* jobz, const char* uplo, const int* n,
                   const int* ka, const int* kb, double* ab, const int* ldab,
                   double* bb, const int* ldbb, double* w, double* z,
                   const int* ldz, double* work, int* info);
extern void dsbtrd_(const char* vect, const char* uplo, const int* n,
                    const int* kd, double* ab, const int* ldab, double* d,
                    double* e, double* q, const int* ldq, double* work,
                    int* info);

extern void dspcon_(const char* uplo, const int* n, const double* ap,
                    const int* ipiv, const double* anorm, double* rcond,
                    double* work, int* iwork, int* info);
extern void dspev_(const char* jobz, const char* uplo, const int* n, double* ap,
                   double* w, double* z, const int* ldz, double* work,
                   int* info);
extern void dspevd_(const char* jobz, const char* uplo, const int* n,
                    double* ap, double* w, double* z, const int* ldz,
                    double* work, const int* lwork, int* iwork,
                    const int* liwork, int* info);
extern void dspevx_(const char* jobz, const char* range, const char* uplo,
                    const int* n, double* ap, const double* vl,
                    const double* vu, const int* il, const int* iu,
                    const double* abstol, int* m, double* w, double* z,
                    const int* ldz, double* work, int* iwork, int* ifail,
                    int* info);
extern void dspgst_(const int* itype, const char* uplo, const int* n,
                    double* ap, double* bp, int* info);
extern void dspgv_(const int* itype, const char* jobz, const char* uplo,
                   const int* n, double* ap, double* bp, double* w, double* z,
                   const int* ldz, double* work, int* info);

extern void dsprfs_(const char* uplo, const int* n, const int* nrhs,
                    const double* ap, const double* afp, const int* ipiv,
                    const double* b, const int* ldb, double* x, const int* ldx,
                    double* ferr, double* berr, double* work, int* iwork,
                    int* info);

extern void dspsv_(const char* uplo, const int* n, const int* nrhs, double* ap,
                   int* ipiv, double* b, const int* ldb, int* info);

extern void dspsvx_(const int* fact, const char* uplo, const int* n,
                    const int* nrhs, const double* ap, double* afp, int* ipiv,
                    const double* b, const int* ldb, double* x, const int* ldx,
                    double* rcond, double* ferr, double* berr, double* work,
                    int* iwork, int* info);

extern void dsptrd_(const char* uplo, const int* n, double* ap, double* d,
                    double* e, double* tau, int* info);

extern void dsptrf_(const char* uplo, const int* n, double* ap, int* ipiv,
                    int* info);

extern void dsptri_(const char* uplo, const int* n, double* ap, const int* ipiv,
                    double* work, int* info);

extern void dsptrs_(const char* uplo, const int* n, const int* nrhs,
                    const double* ap, const int* ipiv, double* b,
                    const int* ldb, int* info);

extern void dstebz_(const char* range, const char* order, const int* n,
                    const double* vl, const double* vu, const int* il,
                    const int* iu, const double* abstol, const double* d,
                    const double* e, int* m, int* nsplit, double* w,
                    int* iblock, int* isplit, double* work, int* iwork,
                    int* info);
extern void dstedc_(const char* compz, const int* n, double* d, double* e,
                    double* z, const int* ldz, double* work, const int* lwork,
                    int* iwork, const int* liwork, int* info);
extern void dstein_(const int* n, const double* d, const double* e,
                    const int* m, const double* w, const int* iblock,
                    const int* isplit, double* z, const int* ldz, double* work,
                    int* iwork, int* ifail, int* info);
extern void dsteqr_(const char* compz, const int* n, double* d, double* e,
                    double* z, const int* ldz, double* work, int* info);
extern void dsterf_(const int* n, double* d, double* e, int* info);
extern void dstev_(const char* jobz, const int* n, double* d, double* e,
                   double* z, const int* ldz, double* work, int* info);
extern void dstevd_(const char* jobz, const int* n, double* d, double* e,
                    double* z, const int* ldz, double* work, const int* lwork,
                    int* iwork, const int* liwork, int* info);
extern void dstevx_(const char* jobz, const char* range, const int* n,
                    double* d, double* e, const double* vl, const double* vu,
                    const int* il, const int* iu, const double* abstol, int* m,
                    double* w, double* z, const int* ldz, double* work,
                    int* iwork, int* ifail, int* info);

extern void dsycon_(const char* uplo, const int* n, const double* a,
                    const int* lda, const int* ipiv, const double* anorm,
                    double* rcond, double* work, int* iwork, int* info);
extern void dsyev_(const char* jobz, const char* uplo, const int* n, double* a,
                   const int* lda, double* w, double* work, const int* lwork,
                   int* info);
extern void dsyevd_(const char* jobz, const char* uplo, const int* n, double* a,
                    const int* lda, double* w, double* work, const int* lwork,
                    int* iwork, const int* liwork, int* info);
extern void dsyevx_(const char* jobz, const char* range, const char* uplo,
                    const int* n, double* a, const int* lda, const double* vl,
                    const double* vu, const int* il, const int* iu,
                    const double* abstol, int* m, double* w, double* z,
                    const int* ldz, double* work, const int* lwork, int* iwork,
                    int* ifail, int* info);
extern void dsyevr_(const char* jobz, const char* range, const char* uplo,
                    const int* n, double* a, const int* lda, const double* vl,
                    const double* vu, const int* il, const int* iu,
                    const double* abstol, int* m, double* w, double* z,
                    const int* ldz, int* isuppz, double* work, const int* lwork,
                    int* iwork, const int* liwork, int* info);
extern void dsygs2_(const int* itype, const char* uplo, const int* n, double* a,
                    const int* lda, const double* b, const int* ldb, int* info);
extern void dsygst_(const int* itype, const char* uplo, const int* n, double* a,
                    const int* lda, const double* b, const int* ldb, int* info);
extern void dsygv_(const int* itype, const char* jobz, const char* uplo,
                   const int* n, double* a, const int* lda, double* b,
                   const int* ldb, double* w, double* work, const int* lwork,
                   int* info);
extern void dsyrfs_(const char* uplo, const int* n, const int* nrhs,
                    const double* a, const int* lda, const double* af,
                    const int* ldaf, const int* ipiv, const double* b,
                    const int* ldb, double* x, const int* ldx, double* ferr,
                    double* berr, double* work, int* iwork, int* info);

extern void dsysv_(const char* uplo, const int* n, const int* nrhs, double* a,
                   const int* lda, int* ipiv, double* b, const int* ldb,
                   double* work, const int* lwork, int* info);

extern void dsysvx_(const int* fact, const char* uplo, const int* n,
                    const int* nrhs, const double* a, const int* lda,
                    double* af, const int* ldaf, int* ipiv, const double* b,
                    const int* ldb, double* x, const int* ldx, double* rcond,
                    double* ferr, double* berr, double* work, const int* lwork,
                    int* iwork, int* info);

extern void dsytd2_(const char* uplo, const int* n, double* a, const int* lda,
                    double* d, double* e, double* tau, int* info);

extern void dsytf2_(const char* uplo, const int* n, double* a, const int* lda,
                    int* ipiv, int* info);

extern void dsytrd_(const char* uplo, const int* n, double* a, const int* lda,
                    double* d, double* e, double* tau, double* work,
                    const int* lwork, int* info);

extern void dsytrf_(const char* uplo, const int* n, double* a, const int* lda,
                    int* ipiv, double* work, const int* lwork, int* info);

extern void dsytri_(const char* uplo, const int* n, double* a, const int* lda,
                    const int* ipiv, double* work, int* info);

extern void dsytrs_(const char* uplo, const int* n, const int* nrhs,
                    const double* a, const int* lda, const int* ipiv, double* b,
                    const int* ldb, int* info);

extern void dtbcon_(const char* norm, const char* uplo, const char* diag,
                    const int* n, const int* kd, const double* ab,
                    const int* ldab, double* rcond, double* work, int* iwork,
                    int* info);
extern void dtbrfs_(const char* uplo, const char* trans, const char* diag,
                    const int* n, const int* kd, const int* nrhs,
                    const double* ab, const int* ldab, const double* b,
                    const int* ldb, double* x, const int* ldx, double* ferr,
                    double* berr, double* work, int* iwork, int* info);
extern void dtbtrs_(const char* uplo, const char* trans, const char* diag,
                    const int* n, const int* kd, const int* nrhs,
                    const double* ab, const int* ldab, double* b,
                    const int* ldb, int* info);

extern void dtgevc_(const char* side, const char* howmny, const int* select,
                    const int* n, const double* a, const int* lda,
                    const double* b, const int* ldb, double* vl,
                    const int* ldvl, double* vr, const int* ldvr, const int* mm,
                    int* m, double* work, int* info);

extern void dtgsja_(const char* jobu, const char* jobv, const char* jobq,
                    const int* m, const int* p, const int* n, const int* k,
                    const int* l, double* a, const int* lda, double* b,
                    const int* ldb, const double* tola, const double* tolb,
                    double* alpha, double* beta, double* u, const int* ldu,
                    double* v, const int* ldv, double* q, const int* ldq,
                    double* work, int* ncycle, int* info);
extern void dtpcon_(const char* norm, const char* uplo, const char* diag,
                    const int* n, const double* ap, double* rcond, double* work,
                    int* iwork, int* info);

extern void dtprfs_(const char* uplo, const char* trans, const char* diag,
                    const int* n, const int* nrhs, const double* ap,
                    const double* b, const int* ldb, double* x, const int* ldx,
                    double* ferr, double* berr, double* work, int* iwork,
                    int* info);

extern void dtptri_(const char* uplo, const char* diag, const int* n,
                    double* ap, int* info);

extern void dtptrs_(const char* uplo, const char* trans, const char* diag,
                    const int* n, const int* nrhs, const double* ap, double* b,
                    const int* ldb, int* info);

extern void dtrcon_(const char* norm, const char* uplo, const char* diag,
                    const int* n, const double* a, const int* lda,
                    double* rcond, double* work, int* iwork, int* info);

extern void dtrevc_(const char* side, const char* howmny, const int* select,
                    const int* n, const double* t, const int* ldt, double* vl,
                    const int* ldvl, double* vr, const int* ldvr, const int* mm,
                    int* m, double* work, int* info);

extern void dtrexc_(const char* compq, const int* n, double* t, const int* ldt,
                    double* q, const int* ldq, int* ifst, int* ILST,
                    double* work, int* info);

extern void dtrrfs_(const char* uplo, const char* trans, const char* diag,
                    const int* n, const int* nrhs, const double* a,
                    const int* lda, const double* b, const int* ldb, double* x,
                    const int* ldx, double* ferr, double* berr, double* work,
                    int* iwork, int* info);

extern void dtrsen_(const char* job, const char* compq, const int* select,
                    const int* n, double* t, const int* ldt, double* q,
                    const int* ldq, double* wr, double* wi, int* m, double* s,
                    double* sep, double* work, const int* lwork, int* iwork,
                    const int* liwork, int* info);

extern void dtrsna_(const char* job, const char* howmny, const int* select,
                    const int* n, const double* t, const int* ldt,
                    const double* vl, const int* ldvl, const double* vr,
                    const int* ldvr, double* s, double* sep, const int* mm,
                    int* m, double* work, const int* lwork, int* iwork,
                    int* info);

extern void dtrsyl_(const char* trana, const char* tranb, const int* isgn,
                    const int* m, const int* n, const double* a, const int* lda,
                    const double* b, const int* ldb, double* c, const int* ldc,
                    double* scale, int* info);

extern void dtrti2_(const char* uplo, const char* diag, const int* n, double* a,
                    const int* lda, int* info);

extern void dtrtri_(const char* uplo, const char* diag, const int* n, double* a,
                    const int* lda, int* info);

extern void dtrtrs_(const char* uplo, const char* trans, const char* diag,
                    const int* n, const int* nrhs, const double* a,
                    const int* lda, double* b, const int* ldb, int* info);

extern void dtzrqf_(const int* m, const int* n, double* a, const int* lda,
                    double* tau, int* info);

extern void dhgeqz_(const char* job, const char* compq, const char* compz,
                    const int* n, const int* ILO, const int* IHI, double* a,
                    const int* lda, double* b, const int* ldb, double* alphar,
                    double* alphai, const double* beta, double* q,
                    const int* ldq, double* z, const int* ldz, double* work,
                    const int* lwork, int* info);
extern void dhsein_(const char* side, const char* eigsrc, const char* initv,
                    int* select, const int* n, double* h, const int* ldh,
                    double* wr, double* wi, double* vl, const int* ldvl,
                    double* vr, const int* ldvr, const int* mm, int* m,
                    double* work, int* ifaill, int* ifailr, int* info);
extern void dhseqr_(const char* job, const char* compz, const int* n,
                    const int* ilo, const int* ihi, double* h, const int* ldh,
                    double* wr, double* wi, double* z, const int* ldz,
                    double* work, const int* lwork, int* info);
extern void dlabad_(double* small, double* large);
extern void dlabrd_(const int* m, const int* n, const int* nb, double* a,
                    const int* lda, double* d, double* e, double* tauq,
                    double* taup, double* x, const int* ldx, double* y,
                    const int* ldy);
extern void dlacon_(const int* n, double* v, double* x, int* isgn, double* est,
                    int* kase);
extern void dlacpy_(const char* uplo, const int* m, const int* n,
                    const double* a, const int* lda, double* b, const int* ldb);
extern void dladiv_(const double* a, const double* b, const double* c,
                    const double* d, double* p, double* q);
extern void dlae2_(const double* a, const double* b, const double* c,
                   double* rt1, double* rt2);
extern void dlaebz_(const int* ijob, const int* nitmax, const int* n,
                    const int* mmax, const int* minp, const int* nbmin,
                    const double* abstol, const double* reltol,
                    const double* pivmin, double* d, double* e, double* e2,
                    int* nval, double* ab, double* c, int* mout, int* nab,
                    double* work, int* iwork, int* info);
extern void dlaed0_(const int* icompq, const int* qsiz, const int* n, double* d,
                    double* e, double* q, const int* ldq, double* qstore,
                    const int* ldqs, double* work, int* iwork, int* info);
extern void dlaed1_(const int* n, double* d, double* q, const int* ldq,
                    int* indxq, const double* rho, const int* cutpnt,
                    double* work, int* iwork, int* info);
extern void dlaed2_(const int* k, const int* n, double* d, double* q,
                    const int* ldq, int* indxq, double* rho, const int* cutpnt,
                    double* z, double* dlamda, double* q2, const int* ldq2,
                    int* indxc, int* w, int* indxp, int* indx, int* coltyp,
                    int* info);
extern void dlaed3_(const int* k, const int* kstart, const int* kstop,
                    const int* n, double* d, double* q, const int* ldq,
                    const double* rho, const int* cutpnt, double* dlamda,
                    int* q2, const int* ldq2, int* indxc, int* ctot, double* w,
                    double* s, const int* lds, int* info);
extern void dlaed4_(const int* n, const int* i, const double* d,
                    const double* z, const double* delta, const double* rho,
                    double* dlam, int* info);
extern void dlaed5_(const int* i, const double* d, const double* z,
                    double* delta, const double* rho, double* dlam);
extern void dlaed6_(const int* kniter, const int* orgati, const double* rho,
                    const double* d, const double* z, const double* finit,
                    double* tau, int* info);
extern void dlaed7_(const int* icompq, const int* n, const int* qsiz,
                    const int* tlvls, const int* curlvl, const int* curpbm,
                    double* d, double* q, const int* ldq, int* indxq,
                    const double* rho, const int* cutpnt, double* qstore,
                    double* qptr, const int* prmptr, const int* perm,
                    const int* givptr, const int* givcol, const double* givnum,
                    double* work, int* iwork, int* info);
extern void dlaed8_(const int* icompq, const int* k, const int* n,
                    const int* qsiz, double* d, double* q, const int* ldq,
                    const int* indxq, double* rho, const int* cutpnt,
                    const double* z, double* dlamda, double* q2,
                    const int* ldq2, double* w, int* perm, int* givptr,
                    int* givcol, double* givnum, int* indxp, int* indx,
                    int* info);
extern void dlaed9_(const int* k, const int* kstart, const int* kstop,
                    const int* n, double* d, double* q, const int* ldq,
                    const double* rho, const double* dlamda, const double* w,
                    double* s, const int* lds, int* info);
extern void dlaeda_(const int* n, const int* tlvls, const int* curlvl,
                    const int* curpbm, const int* prmptr, const int* perm,
                    const int* givptr, const int* givcol, const double* givnum,
                    const double* q, const int* qptr, double* z, double* ztemp,
                    int* info);
extern void dlaein_(const int* rightv, const int* noinit, const int* n,
                    const double* h, const int* ldh, const double* wr,
                    const double* wi, double* vr, double* vi, double* b,
                    const int* ldb, double* work, const double* eps3,
                    const double* smlnum, const double* bignum, int* info);
extern void dlaev2_(const double* a, const double* b, const double* c,
                    double* rt1, double* rt2, double* cs1, double* sn1);
extern void dlaexc_(const int* wantq, const int* n, double* t, const int* ldt,
                    double* q, const int* ldq, const int* j1, const int* n1,
                    const int* n2, double* work, int* info);
extern void dlag2_(const double* a, const int* lda, const double* b,
                   const int* ldb, const double* safmin, double* scale1,
                   double* scale2, double* wr1, double* wr2, double* wi);
extern void dlags2_(const int* upper, const double* a1, const double* a2,
                    const double* a3, const double* b1, const double* b2,
                    const double* b3, double* csu, double* snu, double* csv,
                    double* snv, double* csq, double* snq);
extern void dlagtf_(const int* n, double* a, const double* lambda, double* b,
                    double* c, const double* tol, double* d, int* in,
                    int* info);
extern void dlagtm_(const char* trans, const int* n, const int* nrhs,
                    const double* alpha, const double* dl, const double* d,
                    const double* du, const double* x, const int* ldx,
                    const double* beta, double* b, const int* ldb);
extern void dlagts_(const int* job, const int* n, const double* a,
                    const double* b, const double* c, const double* d,
                    const int* in, double* y, double* tol, int* info);
extern void dlahqr_(const int* wantt, const int* wantz, const int* n,
                    const int* ilo, const int* ihi, double* H, const int* ldh,
                    double* wr, double* wi, const int* iloz, const int* ihiz,
                    double* z, const int* ldz, int* info);
extern void dlahrd_(const int* n, const int* k, const int* nb, double* a,
                    const int* lda, double* tau, double* t, const int* ldt,
                    double* y, const int* ldy);
extern void dlaic1_(const int* job, const int* j, const double* x,
                    const double* sest, const double* w, const double* gamma,
                    double* sestpr, double* s, double* c);
extern void dlaln2_(const int* ltrans, const int* na, const int* nw,
                    const double* smin, const double* ca, const double* a,
                    const int* lda, const double* d1, const double* d2,
                    const double* b, const int* ldb, const double* wr,
                    const double* wi, double* x, const int* ldx, double* scale,
                    double* xnorm, int* info);
extern double dlamch_(const char* cmach);
extern void dlamrg_(const int* n1, const int* n2, const double* a,
                    const int* dtrd1, const int* dtrd2, int* index);
extern double dlangb_(const char* norm, const int* n, const int* kl,
                      const int* ku, const double* ab, const int* ldab,
                      double* work);
extern double dlange_(const char* norm, const int* m, const int* n,
                      const double* a, const int* lda, double* work);
extern double dlangt_(const char* norm, const int* n, const double* dl,
                      const double* d, const double* du);
extern double dlanhs_(const char* norm, const int* n, const double* a,
                      const int* lda, double* work);
extern double dlansb_(const char* norm, const char* uplo, const int* n,
                      const int* k, const double* ab, const int* ldab,
                      double* work);
extern double dlansp_(const char* norm, const char* uplo, const int* n,
                      const double* ap, double* work);
extern double dlanst_(const char* norm, const int* n, const double* d,
                      const double* e);
extern double dlansy_(const char* norm, const char* uplo, const int* n,
                      const double* a, const int* lda, double* work);
extern double dlantb_(const char* norm, const char* uplo, const char* diag,
                      const int* n, const int* k, const double* ab,
                      const int* ldab, double* work);
extern double dlantp_(const char* norm, const char* uplo, const char* diag,
                      const int* n, const double* ap, double* work);
extern double dlantr_(const char* norm, const char* uplo, const char* diag,
                      const int* m, const int* n, const double* a,
                      const int* lda, double* work);
extern void dlanv2_(double* a, double* b, double* c, double* d, double* rt1r,
                    double* rt1i, double* rt2r, double* rt2i, double* cs,
                    double* sn);
extern void dlapll_(const int* n, double* x, const int* incx, double* y,
                    const int* incy, double* ssmin);
extern void dlapmt_(const int* forwrd, const int* m, const int* n, double* x,
                    const int* ldx, const int* k);
extern double dlapy2_(const double* x, const double* y);
extern double dlapy3_(const double* x, const double* y, const double* z);
extern void dlaqgb_(const int* m, const int* n, const int* kl, const int* ku,
                    double* ab, const int* ldab, double* r, double* c,
                    double* rowcnd, double* colcnd, const double* amax,
                    char* equed);
extern void dlaqge_(const int* m, const int* n, double* a, const int* lda,
                    double* r, double* c, double* rowcnd, double* colcnd,
                    const double* amax, char* equed);
extern void dlaqsb_(const char* uplo, const int* n, const int* kd, double* ab,
                    const int* ldab, const double* s, const double* scond,
                    const double* amax, char* equed);
extern void dlaqsp_(const char* uplo, const int* n, double* ap, const double* s,
                    const double* scond, const double* amax, int* equed);
extern void dlaqsy_(const char* uplo, const int* n, double* a, const int* lda,
                    const double* s, const double* scond, const double* amax,
                    int* equed);
extern void dlaqtr_(const int* ltran, const int* lreal, const int* n,
                    const double* t, const int* ldt, const double* b,
                    const double* w, double* scale, double* x, double* work,
                    int* info);
extern void dlar2v_(const int* n, double* x, double* y, double* z,
                    const int* incx, const double* c, const double* s,
                    const int* incc);
extern void dlarf_(const char* side, const int* m, const int* n,
                   const double* v, const int* incv, const double* tau,
                   double* c, const int* ldc, double* work);
extern void dlarfb_(const char* side, const char* trans, const char* direct,
                    const char* storev, const int* m, const int* n,
                    const int* k, const double* v, const int* ldv,
                    const double* t, const int* ldt, double* c, const int* ldc,
                    double* work, const int* lwork);
extern void dlarfg_(const int* n, const double* alpha, double* x,
                    const int* incx, double* tau);
extern void dlarft_(const char* direct, const char* storev, const int* n,
                    const int* k, double* v, const int* ldv, const double* tau,
                    double* t, const int* ldt);
extern void dlarfx_(const char* side, const int* m, const int* n,
                    const double* v, const double* tau, double* c,
                    const int* ldc, double* work);
extern void dlargv_(const int* n, double* x, const int* incx, double* y,
                    const int* incy, double* c, const int* incc);
extern void dlarnv_(const int* idist, int* iseed, const int* n, double* x);
extern void dlartg_(const double* f, const double* g, double* cs, double* sn,
                    double* r);
extern void dlartv_(const int* n, double* x, const int* incx, double* y,
                    const int* incy, const double* c, const double* s,
                    const int* incc);
extern void dlaruv_(int* iseed, const int* n, double* x);

extern void dlas2_(const double* f, const double* g, const double* h,
                   double* ssmin, double* ssmax);

extern void dlascl_(const char* type, const int* kl, const int* ku,
                    double* cfrom, double* cto, const int* m, const int* n,
                    double* a, const int* lda, int* info);

extern void dlaset_(const char* uplo, const int* m, const int* n,
                    const double* alpha, const double* beta, double* a,
                    const int* lda);
extern void dlasq1_(const int* n, double* d, double* e, double* work,
                    int* info);
extern void dlasq2_(const int* m, double* q, double* e, double* qq, double* ee,
                    const double* eps, const double* tol2, const double* small2,
                    double* sup, int* kend, int* info);
extern void dlasq3_(int* n, double* q, double* e, double* qq, double* ee,
                    double* sup, double* sigma, int* kend, int* off,
                    int* iphase, const int* iconv, const double* eps,
                    const double* tol2, const double* small2);
extern void dlasq4_(const int* n, const double* q, const double* e, double* tau,
                    double* sup);
extern void dlasr_(const char* side, const char* pivot, const char* direct,
                   const int* m, const int* n, const double* c, const double* s,
                   double* a, const int* lda);
extern void dlasrt_(const char* id, const int* n, double* d, int* info);
extern void dlassq_(const int* n, const double* x, const int* incx,
                    double* scale, double* sumsq);
extern void dlasv2_(const double* f, const double* g, const double* h,
                    double* ssmin, double* ssmax, double* snr, double* csr,
                    double* snl, double* csl);
extern void dlaswp_(const int* n, double* a, const int* lda, const int* k1,
                    const int* k2, const int* ipiv, const int* incx);
extern void dlasy2_(const int* ltranl, const int* ltranr, const int* isgn,
                    const int* n1, const int* n2, const double* tl,
                    const int* ldtl, const double* tr, const int* ldtr,
                    const double* b, const int* ldb, double* scale, double* x,
                    const int* ldx, double* xnorm, int* info);
extern void dlasyf_(const char* uplo, const int* n, const int* nb,
                    const int* kb, double* a, const int* lda, int* ipiv,
                    double* w, const int* ldw, int* info);
extern void dlatbs_(const char* uplo, const char* trans, const char* diag,
                    const char* normin, const int* n, const int* kd,
                    const double* ab, const int* ldab, double* x, double* scale,
                    double* cnorm, int* info);
extern void dlatps_(const char* uplo, const char* trans, const char* diag,
                    const char* normin, const int* n, const double* ap,
                    double* x, double* scale, double* cnorm, int* info);
extern void dlatrd_(const char* uplo, const int* n, const int* nb, double* a,
                    const int* lda, double* e, double* tau, double* w,
                    const int* ldw);
extern void dlatrs_(const char* uplo, const char* trans, const char* diag,
                    const char* normin, const int* n, const double* a,
                    const int* lda, double* x, double* scale, double* cnorm,
                    int* info);
extern void dlatzm_(const char* side, const int* m, const int* n,
                    const double* v, const int* incv, const double* tau,
                    double* c1, double* c2, const int* ldc, double* work);
extern void dlauu2_(const char* uplo, const int* n, double* a, const int* lda,
                    int* info);
extern void dlauum_(const char* uplo, const int* n, double* a, const int* lda,
                    int* info);

}  // extern "C"

#endif  // THIRD_PARTY_LAPACK_F77_H_
