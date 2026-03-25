#ifndef HWA_BASE_EIGENFUNC_H
#define HWA_BASE_EIGENFUNC_H

#include <Eigen/Eigen>
#include <stdexcept>
#include <vector>

namespace HWA_BASE {
    class Vector;
    class Diagonal;
    class Symmetric;
    class RowProxy;
    class ConstRowProxy;
    class Triple;
    class SO3;
    class SE3;

    namespace detail {
        template<class T>
        constexpr bool is_allowed =
            std::is_same<T, int>::value ||
            std::is_same<T, float>::value ||
            std::is_same<T, double>::value;
    }

    class RowProxy {
        Eigen::Ref<Eigen::MatrixXd> mat_;
        const int row_;
    public:
        RowProxy(Eigen::Ref<Eigen::MatrixXd> m, int r) : mat_(m), row_(r) {}
        double  operator[](int j) const { return mat_(row_, j); }
        double& operator[](int j) { return mat_(row_, j); }
    };

    class ConstRowProxy {
        Eigen::Ref<const Eigen::MatrixXd> mat_;
        int row_;
    public:
        ConstRowProxy(Eigen::Ref<const Eigen::MatrixXd> m, int r) : mat_(m), row_(r) {}
        double operator[](int j) const {
            return mat_(row_, j);
        }
    };

    class Matrix {

    public:
        template<class Derived,
            std::enable_if_t<!std::is_same_v<
            std::decay_t<Derived>,
            Eigen::Ref<Eigen::MatrixXd>>, int> = 0>
        Matrix(const Eigen::MatrixBase<Derived>& eigenMat)   // ·Ç explicit
            : A_(eigenMat), ref_(A_){}
        explicit Matrix(): A_(), ref_(A_) {};
        explicit Matrix(int v, int m, int n) : A_(m, n), ref_(A_) { setConstant(v); }
        explicit Matrix(int m, int n) : A_(m, n), ref_(A_) { A_.setZero(); }
        explicit Matrix(int n) : A_(n, n), ref_(A_) { A_.setZero(); }
        explicit Matrix(Eigen::MatrixXd A): A_(A), ref_(A_) {}
        explicit Matrix(Eigen::Ref<Eigen::MatrixXd> r)
            : ref_(r) {A_ = ref_;}
        virtual ~Matrix() = default;

        operator Eigen::MatrixXd()& { return ref_; }
        operator Eigen::MatrixXd() const& { return ref_; }
        
        class CommaInitializer;
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        CommaInitializer operator<<(T first)
        {
            ref_.setZero();
            ref_(0,0) = static_cast<double>(first);
            return CommaInitializer(*this, 1);
        }
        bool sizeequal(const Matrix& A, const Matrix& B) { bool equal = A.rows() == B.rows() && A.cols() == B.cols(); return equal; }
        bool operator==(const Matrix& D) const { return ref_ == D.ref(); }
        Matrix operator=(const Matrix& D) { if (!sizeequal(*this, D)) { resize(D.rows(), D.cols());}; ref_ = D.ref_; return *this; };
        template <class T, class = std::enable_if_t<detail::is_allowed<T>>>
        operator T() const { assert(ref_.size() == 1); return static_cast<T>(A_(0)); }
        Matrix operator*(const Matrix& D) const {return Matrix(ref_ * D.ref());}
        Matrix operator/(const Matrix& D) const { return *this * D.inverse(); }
        Matrix operator+(const Matrix& D) const {return Matrix(ref_ + D.ref());}
        Matrix operator-(const Matrix& D) const {return Matrix(ref_ - D.ref());}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator*(T b) const { return Matrix(ref_ * static_cast<double>(b)); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator*=(T b) { ref_ = ref_ * static_cast<double>(b); return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator/(T b) const { return Matrix(ref_ / static_cast<double>(b)); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator/=(T b) { ref_ = ref_ / static_cast<double>(b); return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Matrix operator*(T b, const Matrix& a) { return Matrix(a.ref() * static_cast<double>(b)); }

        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator+(T b) const {
            Matrix tmp(rows(), cols());
            for (int i = 0; i < rows(); i++) {
                for (int j = 0; j < cols(); j++) {
                    tmp(i, j) = ref_(i, j) + static_cast<double>(b);
                }
            }
            return tmp;
        }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Matrix operator-(T b) const {
            Matrix tmp(rows(), cols());
            for (int i = 0; i < rows(); i++) {
                for (int j = 0; j < cols(); j++) {
                    tmp(i, j) = ref_(i, j) - static_cast<double>(b);
                }
            }
            return tmp;
        }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Matrix operator+(T b, const Matrix& a) { return (a + b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Matrix operator-(T b, const Matrix& a) { return (a * (-1.0) + b); }

        inline Matrix operator-() const { return Matrix(ref_ * (-1.0)); }
        inline Matrix operator+() const { return Matrix(ref_ * (1.0)); }
        double operator()(int i, int j) const { return ref_(i, j); }
        double& operator()(int i, int j) { return ref_(i, j); }
        void operator<<(const Matrix& A) { ref_ = A.ref(); }
        RowProxy operator[](int i) { return RowProxy(ref_, i); }
        ConstRowProxy operator[](int i) const { return ConstRowProxy(ref_, i); }

        Eigen::MatrixXd matrix() const { return ref_; }
        Eigen::Ref<Eigen::MatrixXd> ref() { return ref_; }
        Eigen::Ref<const Eigen::MatrixXd> ref() const { return ref_; }
        int rows() const { return static_cast<int>(ref_.rows()); }
        int cols() const { return static_cast<int>(ref_.cols()); }
        int size()  const { return static_cast<int>(ref_.size()); }
        void setZero() { ref_.setZero(); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        void setConstant(T a) { ref_.setConstant(a); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        static Matrix Constant(T v, int m, int n) {Matrix tmp(v, m, n);return tmp;}
        static Matrix Ones(int m, int n) {Matrix tmp(1, m, n);return tmp;}
        auto rowwise() { return ref_.rowwise(); }
        double val(int idx_) const {
            assert(idx_ < size());
            const int r = idx_ % rows();
            const int c = idx_ / rows();
            return ref_(r, c);
        }
        double val() const { assert(size()==1); return ref_(0,0); }
        double sum() const { return ref_.sum(); };
        double norm() const { return ref_.norm(); }
        Matrix transpose() { return Matrix(ref_.transpose()); }
        void setIdentity() { A_.setIdentity(); new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);}
        void setmatrix(Eigen::MatrixXd A) { A_ = A; new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);}
        void resize(int m, int n) { A_.resize(m, n); new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);}
        void resize(int n) { A_.resize(n, n); new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);}
        Matrix trace() const { return Matrix(ref_.trace()); }
        Matrix llt() const { return Matrix(ref_.llt()); }
        Matrix matrixL() const{ return Matrix(ref_.llt().matrixL()); }
        Matrix matrixU() const { return Matrix(ref_.llt().matrixU()); }
        Matrix matrixLLT() const { return Matrix(ref_.llt().matrixLLT()); }
        Matrix inverse() const {
            assert(rows() == cols());
            return Matrix(ref_.inverse());
        }
        Matrix eigenvalues() const {Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(ref_); return Matrix(es.eigenvalues());}
        Matrix eigenvectors() const {
            Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(ref_);
            return Matrix(eigenSolver.eigenvectors().real());
        }
        static Matrix Identity(int n) {return Matrix(Eigen::MatrixXd::Identity(n, n));}
        static Matrix Identity(int m,int n) {return Matrix(Eigen::MatrixXd::Identity(m, n));}
        static Matrix Zero(int n) {return Matrix(Eigen::MatrixXd::Zero(n, n));}
        static Matrix Zero(int m, int n) {return Matrix(Eigen::MatrixXd::Zero(m, n));}
        auto col(int n) { return ref_.col(n); }
        auto row(int n) { return ref_.row(n); }
        Matrix block(int start, int end, int row, int col) { return Matrix(Eigen::Ref<Eigen::MatrixXd>(A_.block(start, end, row, col))); }
        const Matrix block(int start, int end, int row, int col) const { return Matrix(A_.block(start, end, row, col)); }
        Matrix leftCols(int col) { return Matrix(Eigen::Ref<Eigen::MatrixXd>(A_.leftCols(col))); }
        const Matrix leftCols(int col) const { return Matrix(A_.leftCols(col)); }
        Matrix rightCols(int col) { return Matrix(Eigen::Ref<Eigen::MatrixXd>(A_.leftCols(col))); }
        const Matrix rightCols(int col) const { return Matrix(A_.leftCols(col)); }
        void Matrix_remRC(int row, int col);
        void Matrix_rem(std::vector<int>& ind);
        void Matrix_addRC(int row, int col);
        void Matrix_remR(int row);
        void Matrix_remR(std::vector<int>& ind);
        static Matrix rotX(double Angle);
        static Matrix rotY(double Angle);
        static Matrix rotZ(double Angle);

        class CommaInitializer
        {
            Matrix& mat_;
            int     idx_;
        public:
            CommaInitializer(Matrix& m, int i) : mat_(m), idx_(i) {}
            template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
            CommaInitializer&& operator,(T x)&&
            {
                if (idx_ >= mat_.size())
                    throw std::out_of_range("Matrix::operator<< too many values");
                const int r = idx_ % mat_.rows();
                const int c = idx_ / mat_.rows();
                mat_.ref_(r,c) = static_cast<double>(x);
                ++idx_;
                return std::move(*this);
            }
            template<class Derived>
            operator Derived& ()&&
            {
                static_assert(std::is_base_of_v<Matrix, Derived>,
                    "Can only convert to derived class of Matrix");
                return static_cast<Derived&>(mat_);
            }
        };

    protected:
        Eigen::MatrixXd A_;
        Eigen::Ref<Eigen::MatrixXd> ref_;
    };

    class Vector : virtual public Matrix {
    public:
        template<typename Derived>
        Vector(const Eigen::MatrixBase<Derived>& v)
            : Matrix(v.rows(), 1) {
            if (v.cols() != 1) throw std::invalid_argument("Need column vector");
            this->A_ = v;
        }
        Vector() : Matrix() {};
        Vector(const Matrix& R) {};
        explicit Vector(int n) : Matrix(n, 1) {}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        explicit Vector(T v, int n) : Matrix(v, n, 1) {}
        explicit Vector(const double* arr, const int n) : Matrix(n, 1) { for (int i = 0; i < n; i++) ref_(i,0) = arr[i]; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        Vector(std::initializer_list<T> lst)
            : Matrix(lst.size(), 1)
        {
            auto it = lst.begin();
            int i = 0;
            while (it != lst.end()) (*this)(i++) = static_cast<double>(*it++);
        }
        explicit Vector(const Eigen::VectorXd& v) : Matrix(v) {}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        static Vector Constant(T v, int m) { return Vector(v, m); }
        static Vector Ones(int m) { return Vector(1, m); }
        static Vector Zero(int m) { return Vector(0, m); }

        operator Eigen::VectorXd() const { return ref_; }
        double& operator[](int i) { return ref_(i,0); }
        double operator[](int i) const { return ref_(i,0); }
        double& operator()(int i) { return ref_(i,0); }
        double operator()(int i) const { return ref_(i,0); }

        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Vector operator*=(T b) { ref_ *= b; return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Vector operator/=(T b) { ref_ /= b; return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Vector operator/(T b) const { return Vector(ref_ / b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Vector operator*(T b) const { return Vector(ref_ * b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Vector operator*(T b, const Vector& a) { return Vector(a.ref() * b); }
        inline Vector operator-() const { return Vector(ref_*(-1)); }
        Vector operator=(const Eigen::VectorXd& D) { ref_ = D; return *this; };
        Vector operator=(const Matrix& D) { if (!sizeequal(*this, D)) resize(D.rows()); assert(D.cols() == 1); ref_ = D.ref(); return *this; }
        Vector operator+(const Matrix& D) const { assert(D.size() == size()); return Vector(ref_ + D.ref()); }
        Vector operator-(const Matrix& D) const { assert(D.size() == size()); return Vector(ref_ - D.ref()); }
        Matrix operator*(const Matrix& D) const { assert(D.rows() == 1); return Matrix(ref_ * D.ref()); }
        friend inline Matrix operator*(const Matrix& b, const Vector& a) { assert(b.cols() == a.size()); return Matrix(b.ref() * a.ref());}

        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        void fill(T v) { ref_.setConstant(v); }
        auto head(int m) { return Eigen::Ref<Eigen::VectorXd>(ref_).head(m); }
        auto tail(int m) { return Eigen::Ref<Eigen::VectorXd>(ref_).tail(m); }
        auto head(int m) const { return Eigen::Ref<Eigen::VectorXd>(ref_).head(m); }
        auto tail(int m) const { return Eigen::Ref<Eigen::VectorXd>(ref_).tail(m); }
        void resize(int n) { A_.resize(n, 1); new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_); }
        void insert(int pos, double value) {
            Eigen::VectorXd tmp = A_;
            tmp.conservativeResize(tmp.size() + 1, Eigen::NoChange);
            tmp.tail(tmp.size() - pos - 1) = tmp.segment(pos, tmp.size() - pos - 1);
            tmp(pos) = value;
            A_ = std::move(tmp);
            new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);
        }
        void erase(int pos) {
            Eigen::VectorXd tmp = A_;
            tmp.segment(pos, tmp.size() - pos - 1) = tmp.segment(pos + 1, tmp.size() - pos - 1);
            tmp.conservativeResize(tmp.size() - 1, Eigen::NoChange);
            A_ = std::move(tmp);
            new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);
        }
        void addR(int row);
    };

    class Triple : virtual public Vector {
    public:
        template<typename Derived>
        Triple(const Eigen::MatrixBase<Derived>& eigenVec)
            : Vector(eigenVec) {
            if (eigenVec.size() != 3) throw std::invalid_argument("Need 3¡Á1 vector");
        }

        Triple() : Vector(3) {};
        Triple(const Matrix& R) {};
        Triple(const Vector& R) {};
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        explicit Triple(T v) : Vector(v, 3) {};
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        Triple(std::initializer_list<T> lst)
            : Vector(3)
        {
            if (lst.size() != 3)
                throw std::invalid_argument("Triple needs exactly 3 elements");
            auto it = lst.begin();
            (*this)(0) = static_cast<double>(*it++);
            (*this)(1) = static_cast<double>(*it++);
            (*this)(2) = static_cast<double>(*it);
        }
        template<class T0, class T1, class T2,
            std::enable_if_t<detail::is_allowed<std::common_type_t<T0, T1, T2>>, int> = 0>
        Triple(const T0 a0, const T1 a1, const T2 a2) : Vector(3) {
            (*this)(0) = static_cast<double>(a0);
            (*this)(1) = static_cast<double>(a1);
            (*this)(2) = static_cast<double>(a2);
        }
        explicit Triple(const double* arr) : Vector(3) { for (int i = 0; i < 3; i++) ref_(i,0) = arr[i]; }
        explicit Triple(const Eigen::Vector3d& v) : Vector(v) {}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        static Triple Constant(T v) { return Triple(v); }
        static Triple Ones() { return Triple(1); }
        static Triple Zero() { return Triple(0); }

        operator Eigen::Vector3d() const { return ref_; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Triple operator*=(T b) { ref_ *= b; return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Triple operator/=(T b) { ref_ /= b; return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Triple operator/(T b) const { return Triple(ref_ / b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Triple operator*(T b) const { return Triple(ref_ * b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Triple operator*(T b, const Triple& a) { return Triple(a.ref() * b); }
        Triple operator=(const Matrix& D) { ref_ = D.ref(); return *this; }
        Triple operator+(const Triple& D) const { return Triple(ref_ + D.ref()); }
        Triple operator-(const Triple& D) const { return Triple(ref_ - D.ref()); }
        Matrix operator*(const Matrix& D) const { assert(D.rows() == 1); return Matrix(ref_ * D.ref()); }
        inline Triple operator-() const { return Triple(ref_*(-1)); }
        Triple cross(const Triple& b) const {
            Triple c;
            c[0] = (*this)[1] * b[2] - (*this)[2] * b[1];
            c[1] = (*this)[2] * b[0] - (*this)[0] * b[2];
            c[2] = (*this)[0] * b[1] - (*this)[1] * b[0];
            return c;
        }
    };

    class SO3 : public Matrix {
    public:
        SO3(const Matrix& R){};
        SO3() : Matrix(3, 3) {};
        explicit SO3(Eigen::Ref<Eigen::MatrixXd> r)
            : Matrix(r) {}
        SO3 operator=(const Matrix& D) { if (!sizeequal(*this, D)) resize(D.rows(),D.cols()); ref_ = D.ref(); return *this; };
        static SO3 Identity() { SO3 tmp; tmp.setmatrix(Eigen::MatrixXd::Identity(3, 3)); return tmp; }
        static SO3 Zero() { return SO3(); }

    };

    class SE3 : public Matrix {
    public:
        SE3() : Matrix(4, 4) {};
        SE3(const Matrix& R) {};
        explicit SE3(Eigen::Ref<Eigen::MatrixXd> r)
            : Matrix(r) {}
        SE3 operator=(const Matrix& D) { if (!sizeequal(*this, D)) resize(D.rows(), D.cols()); ref_ = D.ref(); return *this; };
        static SE3 Identity() { SE3 tmp; tmp.block(0, 0, 3, 3) = Matrix::Identity(3, 3); return tmp;}
        static SE3 Zero() { return SE3(); }
        SO3 linear() {return SO3(Eigen::Ref<Eigen::MatrixXd>(A_.block(0, 0, 3, 3)));}
        Triple translation() { return Triple(Eigen::Ref<Eigen::MatrixXd>(A_.block(0, 3, 3, 1))); }
    };

    class Symmetric : public Matrix {
    public:
        Symmetric() : Matrix() {};
        Symmetric(const Matrix& R) {};
        explicit Symmetric(int n) : Matrix(n, n) {};

        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Symmetric operator*=(T b) { ref_ *= b; return *this; };
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Symmetric operator/=(T b) { ref_ /= b; return *this; };
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Symmetric operator/(T b) const {
            Symmetric tmp;
            tmp.setmatrix(ref_ / b);
            return tmp;
        };
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Symmetric operator*(T b) const {
            Symmetric tmp;
            tmp.setmatrix(ref_ * b);
            return tmp;
        };
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Symmetric operator*(T b, const Symmetric& a) { return Symmetric(a.matrix() * b); }
        Symmetric operator=(const Matrix& rhs) { if (!sizeequal(*this, rhs)) resize(rhs.rows(), rhs.cols()); ref_ = rhs.ref();}
        Symmetric operator=(const Symmetric& rhs) { if (!sizeequal(*this, rhs)) resize(rhs.rows(), rhs.cols()); ref_ = rhs.ref(); return *this;}
        Symmetric operator+=(double alpha) { ref_.diagonal().array() += alpha; return *this; }
        Symmetric rank1Update(const Eigen::VectorXd& v, double beta = 1.0) { ref_.selfadjointView<Eigen::Lower>().rankUpdate(v, beta);return *this; }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        void set(T v, int i, int j) { ref_(i, j) = static_cast<double>(v); if (i != j) ref_(j, i) = static_cast<double>(v); }
        template<class Derived>
        void setLower(const Eigen::MatrixBase<Derived>& L) {
            const int n = static_cast<int>(L.rows());
            resize(n, n);
            A_.template triangularView<Eigen::Lower>() = L;
            A_.template triangularView<Eigen::StrictlyUpper>() = A_.template triangularView<Eigen::StrictlyLower>().transpose();
            new (&ref_) Eigen::Ref<Eigen::MatrixXd>(A_);
        }
        void Matrix_remRC(int row, int col);
        void Matrix_rem(std::vector<int>& ind);
        void Matrix_addRC(int row, int col);
        void Matrix_swap(int a, int b);
        void Matrix_cpRC(const Symmetric& Q2, int r, int c);
        Symmetric cov2corr()
        {
            Symmetric C(rows());
            for (int i = 0; i < rows(); i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    if (i == j)
                        C(i, j) = 1;
                    assert(ref_(i, i) > 0 && ref_(j, j) > 0);
                    C(i, j) = ref_(i, j) / sqrt(ref_(i, i) * ref_(j, j));
                }
            }
            return C;
        }
    };

    class Diagonal : public Matrix {
    public:
        Diagonal() : Matrix() {};
        Diagonal(const Matrix& R) {};
        explicit Diagonal(int n): Matrix(n,n){}
        explicit Diagonal(Vector V) : Matrix(V.size()){ ref_.diagonal() = V.matrix();}
        const Vector diagonal() const {return Vector(ref_.diagonal());}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        void set(T v, int m) { ref_(m, m) = static_cast<double>(v);}
        void set(Vector V) { resize(V.size(), V.size()); ref_.setZero(); ref_.diagonal() = V.matrix();}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Diagonal operator*=(T b) { ref_.diagonal() *= b; return *this;}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Diagonal operator/=(T b) { ref_.diagonal() /= b; return *this;}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Diagonal operator/(T b) const { return Diagonal(diagonal() / b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        inline Diagonal operator*(T b) const { return Diagonal(diagonal() * b); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        friend inline Diagonal operator*(T b, const Diagonal& a) { return Diagonal(a.diagonal() * b); }
        Diagonal operator=(const Matrix& _D) { set(_D.matrix().diagonal()); return *this; };
        Diagonal operator=(const Diagonal& _D) { if (!sizeequal(*this, _D)) resize(_D.rows(), _D.cols()); ref_ = _D.ref(); return *this; };
        Diagonal SR() const{
            Diagonal SR(rows());  
            for (int i = 0; i < rows(); i++) {
                double tmp = ref_(i, i);
                assert(tmp >= 0);
                SR(i, i) = sqrt(tmp);
            } 
            return SR;
        }
        void Matrix_remRC(int row);
        void Matrix_addRC(int row);
    };
}

#endif
