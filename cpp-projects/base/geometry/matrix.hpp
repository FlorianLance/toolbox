

/*******************************************************************************
** Toolbox-base                                                               **
** MIT License                                                                **
** Copyright (c) [2018] [Florian Lance]                                       **
**                                                                            **
** Permission is hereby granted, free of charge, to any person obtaining a    **
** copy of this software and associated documentation files (the "Software"), **
** to deal in the Software without restriction, including without limitation  **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
** and/or sell copies of the Software, and to permit persons to whom the      **
** Software is furnished to do so, subject to the following conditions:       **
**                                                                            **
** The above copyright notice and this permission notice shall be included in **
** all copies or substantial portions of the Software.                        **
**                                                                            **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
** DEALINGS IN THE SOFTWARE.                                                  **
**                                                                            **
********************************************************************************/

#pragma once

// std
#include <algorithm>
#include <ostream>

// local
#include "utility/math.hpp"
#include "geometry/point.hpp"

namespace tool::geo {

template<typename acc, int _rows, int _cols>
struct Matrix;

// functions
template <typename acc, int _rows, int _cols>
constexpr Matrix<acc,_cols,_rows> transpose(const Matrix<acc,_rows,_cols> &m);
template <typename acc, int _rows, int _cols>
constexpr Matrix<acc,_rows-1,_cols-1> cut(const Matrix<acc,_rows,_cols> &m, int row, int col);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> cofactor(const Matrix<acc,_rows,_cols> &minor);
template <typename acc, int _rows, int _cols>
Matrix<acc,_cols,_rows> adjugate(const Matrix<acc,_rows,_cols> &mat);
template <typename acc, int _rows, int _cols>
bool equals(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r, int ulp = 1);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> add(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> add(const Matrix<acc,_rows,_cols> &m, acc value);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> substract(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> substract(const Matrix<acc,_rows,_cols> &m, acc value);
template <typename acc, int _rowsL, int _colsL, int _rowsR, int _colsR>
Matrix<acc,_rowsL, _colsR> multiply(const Matrix<acc,_rowsL,_colsL> &l, const Matrix<acc,_rowsR,_colsR> &r);
template <typename acc, int _rowsL, int _colsL, int _rowsR>
Vec<acc,_rowsR> multiply(const Matrix<acc,_rowsL,_colsL> &l, const Vec<acc,_rowsR> &r);
template <typename acc, int _rowsL, int _rowsR, int _colsR>
Vec<acc,_rowsR> multiply(const Vec<acc,_rowsL> &l, const Matrix<acc,_rowsR,_colsR> &r);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> multiply(const Matrix<acc,_rows,_cols> &m, acc value);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> divide(const Matrix<acc,_rows,_cols> &m, acc value);

// operators
template <typename acc, int _rows, int _cols>
std::ostream &operator<<(std::ostream &flux, const Matrix<acc, _rows, _cols> &m);
template <typename acc, int _rows, int _cols>
bool operator==(const Matrix<acc, _rows, _cols> &m1, const Matrix<acc, _rows, _cols> &m2);
template <typename acc, int _rows, int _cols>
bool operator!=(const Matrix<acc, _rows, _cols> &m1, const Matrix<acc, _rows, _cols> &m2);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator+(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator+(const Matrix<acc,_rows,_cols> &m, acc value);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator-(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator-(const Matrix<acc,_rows,_cols> &m, acc value);
template <typename acc, int _rowsL, int _colsL, int _rowsR, int _colsR>
Matrix<acc,_rowsL, _colsR> operator*(const Matrix<acc,_rowsL,_colsL> &l, const Matrix<acc,_rowsR,_colsR> &r);
template <typename acc, int _rowsL, int _colsL, int _rowsR>
Vec<acc,_rowsR> operator*(const Matrix<acc,_rowsL,_colsL> &l, const Vec<acc,_rowsR> &r);
template <typename acc, int _rowsL, int _rowsR, int _colsR>
Vec<acc,_rowsR> operator*(const Vec<acc,_rowsL> &l, const Matrix<acc,_rowsR,_colsR> &r);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator*(const Matrix<acc,_rows,_cols> &m, acc value);
template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator/(const Matrix<acc,_rows,_cols> &m, acc value);


template<typename acc, int _rows, int _cols>
struct Matrix{

    static_assert((_rows != 0) && (_cols != 0), "Mat must have a number of rows and columns > 0" );

    Matrix() = default;
    Matrix(const Matrix& other) = default;
    Matrix& operator=(const Matrix& other) = default;
    Matrix(Matrix&& other) = default;
    Matrix& operator=(Matrix&& other) = default;

    constexpr explicit Matrix(bool identity){
        if(identity){
            *this = Matrix::identity();
        }
    }

    //    explicit inline Matrix(acc initValue) noexcept{
    //        std::fill(std::begin(array), std::end(array), initValue);
    //    }

    explicit inline Matrix(acc *values) noexcept{
        std::copy(values, values + _rows*_cols, std::begin(array));
    }

    inline Matrix(std::initializer_list<acc> l) noexcept{
        std::move(l.begin(), l.end(), std::begin(array));
    }

    constexpr Matrix(vecN<acc, _rows*_cols> array) noexcept{
        array = std::move(array);
    }

    // functions
    static Matrix identity() noexcept{
        Matrix m;
        std::fill(std::begin(m.array), std::end(m.array), acc{0});
        if(_rows == _cols){
            for(int ii = 0; ii < _rows; ++ii){
                m.at(ii,ii) = acc{1};
            }
        }
        return m;
    }

    constexpr int rows() const noexcept{return _rows;}
    constexpr int cols() const noexcept{return _cols;}

    constexpr Vec<acc,_cols> row(int row) const{
        Vec<acc,_cols> r;
        const int start = row* _cols;
        std::copy(std::begin(array) + start, std::begin(array) + start + _cols, std::begin(r.v));
        return r;
    }

    constexpr Vec<acc,_rows> col(int col) const{
        Vec<acc,_rows> c;
        for(int ii = 0; ii < _rows; ++ii){
            c[ii] = at(ii, col);
        }
        return c;
    }

    template <typename acc2>
    constexpr Matrix<acc2, _rows, _cols> conv() const{
        Matrix<acc2, _rows, _cols> mc;
        for(size_t ii = 0; ii < _rows*_cols; ++ii){
            mc.array[ii] = static_cast<acc2>(array[ii]);
        }
        return mc;
    }

    constexpr acc at(int id) const {return (*this)(id);}
    inline acc& at(int id) {return (*this)(id);}

    constexpr acc at(int row, int col) const {return (*this)(row,col);}
    inline acc& at(int row, int col) {return (*this)(row,col);}

    constexpr vecN<acc, _rows*_cols> mat() const noexcept{return array;}
    inline vecN<acc, _rows*_cols> &mat() noexcept{return array;}

    // operators
    constexpr acc operator()(int id)  const  { return array[id];}    
    constexpr acc& operator()(int id) {return array[id];}
    constexpr acc operator()(int row, int col)  const{ return array[row*_cols + col];}
    constexpr acc& operator()(int row, int col) {return array[row*_cols + col];}

    inline Matrix<acc,_rows,_cols> operator-(const Matrix<acc,_rows,_cols> &m) const noexcept{
        return multiply(m,acc{-1});
    }

    template <int _rowsR, int _colsR>
    inline Matrix<acc,_rows,_cols> operator+=(const Matrix<acc,_rowsR,_colsR> &r){
        return (*this) = (*this)+r;
    }

    template <int _rowsR>
    inline Matrix<acc,_rows,_cols> operator+=(const Vec<acc,_rowsR> &r){
        return (*this) = (*this)+r;
    }

    inline Matrix<acc,_rows,_cols> operator+=(acc v){
        return (*this) = (*this)+v;
    }

    template <int _rowsR, int _colsR>
    inline Matrix<acc,_rows,_cols> operator-=(const Matrix<acc,_rowsR,_colsR> &r){
        return (*this) = (*this)-r;
    }

    template <int _rowsR>
    inline Matrix<acc,_rows,_cols> operator-=(const Vec<acc,_rowsR> &r){
        return (*this) = (*this)-r;
    }

    inline Matrix<acc,_rows,_cols> operator-=(acc v){
        return (*this) = (*this)-v;
    }

    template <int _rowsR, int _colsR>
    inline Matrix<acc,_rows,_cols> operator*=(const Matrix<acc,_rowsR,_colsR> &r){
        return (*this) = (*this)*r;
    }

    template <int _rowsR>
    inline Matrix<acc,_rows,_cols> operator*=(const Vec<acc,_rowsR> &r){
        return (*this) = (*this)*r;
    }

    inline Matrix<acc,_rows,_cols> operator*=(acc v){
        return (*this) = (*this)*v;
    }   

    vecN<acc, _rows*_cols> array{};
};


template <typename acc, int _rows, int _cols>
constexpr Matrix<acc,_cols,_rows> transpose(const Matrix<acc,_rows,_cols> &m){
    Matrix<acc,_cols,_rows> dest;
    for(int ii = 0; ii < _rows*_cols; ++ii){
        dest.array[ii] = m.array[_rows*(ii % _rows) + (ii / _rows)];
    }
    return dest;
}

template <typename acc, int _rows, int _cols>
constexpr Matrix<acc,_rows-1,_cols-1> cut(const Matrix<acc,_rows,_cols> &m, int row, int col){

    Matrix<acc,_rows-1,_cols-1> res;
    int index = 0;
    for(int ii = 0; ii < _rows; ++ii){
        for(int jj = 0; jj < _cols; ++jj){
            if(ii == row || jj == col){
                continue;
            }
            res.array[index++] = m.array[_cols*ii+jj];
        }
    }

    return res;
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> cofactor(const Matrix<acc,_rows,_cols> &minor){
    Matrix<acc,_rows,_cols> res;
    for(int ii = 0; ii < _rows; ++ii){
        for(int jj = 0; jj < _cols; ++jj){
            int t = _cols * jj +ii;
            int s = _cols * jj +ii;
            acc sign = std::pow(acc{-1}, ii + jj); // + or -
            res.at(t) = minor.at(s)*sign;
        }
    }
    return res;
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_cols,_rows> adjugate(const Matrix<acc,_rows,_cols> &mat){
    return cofactor(mat).transpose();
}

template <typename acc, int _rows, int _cols>
bool equals(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r, int ulp){
    return std::equal(std::cbegin(l.array), std::cend(l.array), std::cbegin(r.array),
        [ulp](const acc &v1, const acc &v2) -> bool{
            return almost_equal<acc>(v1,v2,ulp);
        }
    );
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> add(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r){
    Matrix<acc,_rows,_cols> res;
    std::transform(l.array.cbegin(), l.array.cend(), r.array.cbegin(), std::begin(res.array), std::plus<acc>());
    return res;
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> add(const Matrix<acc,_rows,_cols> &m, acc value){
    using namespace std::placeholders;
    Matrix<acc,_rows,_cols> res;
    std::transform(m.array.cbegin(), m.array.cend(), std::begin(res.array), std::bind(std::plus<acc>(), _1, value));
    return res;
}


template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> substract(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r){
    Matrix<acc,_rows,_cols> res;
    std::transform(l.array.cbegin(), l.array.cend(), r.array.cbegin(), std::begin(res.array), std::minus<acc>());
    return res;
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> substract(const Matrix<acc,_rows,_cols> &m, acc value){
    using namespace std::placeholders;
    Matrix<acc,_rows,_cols> res;
    std::transform(m.array.cbegin(), m.array.cend(), std::begin(res.array), std::bind(std::minus<acc>(), _1, value));
    return res;
}

template <typename acc, int _rowsL, int _colsL, int _rowsR, int _colsR>
Matrix<acc,_rowsL, _colsR> multiply(const Matrix<acc,_rowsL,_colsL> &l, const Matrix<acc,_rowsR,_colsR> &r){
    Matrix<acc,_rowsL, _colsR> res;
    for(int ii = 0; ii < l.rows(); ++ii){
        for(int jj = 0; jj < r.cols(); ++jj){
            for(int kk = 0; kk < r.rows(); ++kk){
                res(r.cols() * ii +jj) += l(l.cols() * ii + kk) * r(r.cols() * kk + jj);
            }
        }
    }
    return res;
}

template <typename acc, int _rowsL, int _colsL, int _rowsR>
Vec<acc,_rowsR> multiply(const Matrix<acc,_rowsL,_colsL> &l, const Vec<acc,_rowsR> &r){
    Vec<acc,_rowsR> res;
    for(int ii = 0; ii < _rowsL; ++ii){
        for(int jj = 0; jj < _rowsR; ++jj){
            res[ii] += l(_colsL * ii + jj) * r[jj];
        }
    }
    return res;
}

template <typename acc, int _rowsL, int _rowsR, int _colsR>
Vec<acc,_rowsR> multiply(const Vec<acc,_rowsL> &l, const Matrix<acc,_rowsR,_colsR> &r){

    Vec<acc,_rowsL> res;
    for(int ii = 0; ii < _rowsL; ++ii){
        for(int jj = 0; jj < _rowsR; ++jj){
            res[ii] += l[ii + jj] * r[jj];
        }
    }

    return res;
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> multiply(const Matrix<acc,_rows,_cols> &m, acc value){
    using namespace std::placeholders;
    Matrix<acc,_rows,_cols> res;
    std::transform(m.array.cbegin(), m.array.cend(), std::begin(res.array), std::bind(std::multiplies<acc>(), _1, value));
    return res;
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> divide(const Matrix<acc,_rows,_cols> &m, acc value){
    if(value > acc{0}){
        using namespace std::placeholders;
        Matrix<acc,_rows,_cols> res;
        std::transform(m.array.cbegin(), m.array.cend(), std::begin(res.array), std::bind(std::divides<acc>(), _1, value));
        return res;
    }
    return m;
}

template <typename acc, int _rows, int _cols>
std::ostream &operator<<(std::ostream &flux, const Matrix<acc, _rows, _cols> &m){

    for(int ii = 0; ii < _rows; ++ii){
        flux << "[ ";
        for(int jj = 0; jj < _cols; ++jj){
            flux << m.array[ii*_cols + jj] << " ";
        }
        flux << "]\n";
    }
    return flux;
}

template <typename acc, int _rows, int _cols>
bool operator==(const Matrix<acc, _rows, _cols> &m1, const Matrix<acc, _rows, _cols> &m2){
    return equals(m1,m2);
}

template <typename acc, int _rows, int _cols>
bool operator!=(const Matrix<acc, _rows, _cols> &m1, const Matrix<acc, _rows, _cols> &m2){
    return !equals(m1,m2);
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator+(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r){
    return add(l,r);
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator+(const Matrix<acc,_rows,_cols> &m, acc value){
    return add(m, value);
}


template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator-(const Matrix<acc,_rows,_cols> &l, const Matrix<acc,_rows,_cols> &r){
    return substract(l,r);
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator-(const Matrix<acc,_rows,_cols> &m, acc value){
    return substract(m,value);
}

template <typename acc, int _rowsL, int _colsL, int _rowsR, int _colsR>
Matrix<acc,_rowsL, _colsR> operator*(const Matrix<acc,_rowsL,_colsL> &l, const Matrix<acc,_rowsR,_colsR> &r){
    return multiply(l,r);
}

template <typename acc, int _rowsL, int _colsL, int _rowsR>
Vec<acc,_rowsR> operator*(const Matrix<acc,_rowsL,_colsL> &l, const Vec<acc,_rowsR> &r){
    return multiply(l,r);
}

template <typename acc, int _rowsL, int _rowsR, int _colsR>
Vec<acc,_rowsR> operator*(const Vec<acc,_rowsL> &l, const Matrix<acc,_rowsR,_colsR> &r){
    return multiply(l,r);
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator*(const Matrix<acc,_rows,_cols> &m, acc value){
    return multiply(m,value);
}

template <typename acc, int _rows, int _cols>
Matrix<acc,_rows,_cols> operator/(const Matrix<acc,_rows,_cols> &m, acc value){
    return divide(m,value);
}



}
