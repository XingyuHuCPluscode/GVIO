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

    class Matrix : public Eigen::MatrixXd {

    public:
        Matrix() :Eigen::MatrixXd() {};
        Matrix(const Eigen::MatrixXd&) {};
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        explicit Matrix(T v, int m, int n) : Eigen::MatrixXd(m,n){ this->setConstant(static_cast<double>(v)); }
        explicit Matrix(int m, int n) : Eigen::MatrixXd(Eigen::MatrixXd::Zero(m, n)){}
        explicit Matrix(int n) : Eigen::MatrixXd(Eigen::MatrixXd::Zero(n, n)){}
        virtual ~Matrix() = default;

        void Matrix_remRC(int row, int col);
        void Matrix_rem(std::vector<int>& ind);
        void Matrix_addRC(int row, int col);
        void Matrix_remR(int row);
        void Matrix_remR(std::vector<int>& ind);
        static Matrix rotX(double Angle);
        static Matrix rotY(double Angle);
        static Matrix rotZ(double Angle);
    };

    class Vector : public Eigen::VectorXd {
    public:
        Vector() : Eigen::VectorXd() {};
        Vector(const Eigen::VectorXd& R) {};
        explicit Vector(int n) : Eigen::VectorXd(Eigen::VectorXd::Zero(n)) {}
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        explicit Vector(T v, int n) : Eigen::VectorXd(n) { this->setConstant(static_cast<double>(v)); }
        template<class T, class = std::enable_if_t<detail::is_allowed<T>>>
        explicit Vector(const T* arr, const int n) : Eigen::VectorXd(n) { for (int i = 0; i < n; i++) (*this)(i) = static_cast<double>(arr[i]); }

        void addR(int row);
    };

    class Triple : public Vector {
    public:
        Triple() : Vector(3) {};
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
        explicit Triple(const double* arr) : Vector(3) { for (int i = 0; i < 3; i++) (*this)(i) = arr[i]; }
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
        SO3(const Matrix& R) { assert(R.rows() == R.cols() && R.rows() == 3); };
        SO3(const Eigen::MatrixXd& R) { assert(R.rows() == R.cols() && R.rows() == 3); };
        SO3() : Matrix(3, 3) {};
        static SO3 Identity() { SO3 tmp = Matrix::Identity(3, 3); return tmp; }
        static SO3 Zero() { return SO3(); }
    };

    class SE3 : public Matrix {
    public:
        SE3() : Matrix(4, 4) {};
        SE3(const Matrix& R) {};
        SE3(const Eigen::MatrixXd& R) { assert(R.rows() == R.cols() && R.rows() == 4); };
        static SE3 Identity() { SE3 tmp = Matrix::Identity(4, 4); return tmp;}
        static SE3 Zero() { return SE3(); }
        SO3 linear() {return this->block(0, 0, 3, 3);}
        Triple translation() { return this->block(0, 3, 3, 1);}
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
