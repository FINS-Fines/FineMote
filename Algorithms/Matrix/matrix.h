/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef MATRIX_PORTABLE_H
#define MATRIX_PORTABLE_H

#include <cmath>
#include <cstring>

// Platform-independent fixed-size float matrix.
template <int _rows, int _cols>
class Matrixf {
 public:
  Matrixf(void) : rows_(_rows), cols_(_cols), data_{} {
    static_assert(_rows > 0 && _cols > 0, "Matrix dimensions must be positive");
  }

  Matrixf(float data[_rows * _cols]) : Matrixf() {
    std::memcpy(this->data_, data, _rows * _cols * sizeof(float));
  }

  Matrixf(const Matrixf<_rows, _cols>& mat) : Matrixf() {
    std::memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
  }

  ~Matrixf(void) {}

  int rows(void) { return _rows; }
  int cols(void) { return _cols; }

  float* operator[](const int& row) { return &this->data_[row * _cols]; }
  const float* operator[](const int& row) const {
    return &this->data_[row * _cols];
  }

  Matrixf<_rows, _cols>& operator=(const Matrixf<_rows, _cols> mat) {
    std::memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
    return *this;
  }

  Matrixf<_rows, _cols>& operator+=(const Matrixf<_rows, _cols> mat) {
    for (int i = 0; i < _rows * _cols; ++i) {
      this->data_[i] += mat.data_[i];
    }
    return *this;
  }

  Matrixf<_rows, _cols>& operator-=(const Matrixf<_rows, _cols> mat) {
    for (int i = 0; i < _rows * _cols; ++i) {
      this->data_[i] -= mat.data_[i];
    }
    return *this;
  }

  Matrixf<_rows, _cols>& operator*=(const float& val) {
    for (int i = 0; i < _rows * _cols; ++i) {
      this->data_[i] *= val;
    }
    return *this;
  }

  Matrixf<_rows, _cols>& operator/=(const float& val) {
    return *this *= 1.0f / val;
  }

  Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols>& mat) {
    Matrixf<_rows, _cols> res(*this);
    res += mat;
    return res;
  }

  Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols>& mat) {
    Matrixf<_rows, _cols> res(*this);
    res -= mat;
    return res;
  }

  Matrixf<_rows, _cols> operator*(const float& val) {
    Matrixf<_rows, _cols> res(*this);
    res *= val;
    return res;
  }

  friend Matrixf<_rows, _cols> operator*(
      const float& val, const Matrixf<_rows, _cols>& mat) {
    Matrixf<_rows, _cols> res(mat);
    res *= val;
    return res;
  }

  Matrixf<_rows, _cols> operator/(const float& val) {
    Matrixf<_rows, _cols> res(*this);
    res /= val;
    return res;
  }

  template <int cols>
  friend Matrixf<_rows, cols> operator*(const Matrixf<_rows, _cols>& mat1,
                                        const Matrixf<_cols, cols>& mat2) {
    Matrixf<_rows, cols> res;
    for (int row = 0; row < _rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        float sum = 0.0f;
        for (int k = 0; k < _cols; ++k) {
          sum += mat1[row][k] * mat2[k][col];
        }
        res[row][col] = sum;
      }
    }
    return res;
  }

  template <int rows, int cols>
  Matrixf<rows, cols> block(const int& start_row, const int& start_col) {
    Matrixf<rows, cols> res;
    for (int row = start_row; row < start_row + rows; ++row) {
      std::memcpy(res[0] + (row - start_row) * cols,
                  this->data_ + row * _cols + start_col,
                  cols * sizeof(float));
    }
    return res;
  }

  Matrixf<1, _cols> row(const int& row) {
    return block<1, _cols>(row, 0);
  }

  Matrixf<_rows, 1> col(const int& col) {
    return block<_rows, 1>(0, col);
  }

  Matrixf<_cols, _rows> trans(void) {
    Matrixf<_cols, _rows> res;
    for (int row = 0; row < _rows; ++row) {
      for (int col = 0; col < _cols; ++col) {
        res[col][row] = (*this)[row][col];
      }
    }
    return res;
  }

  float trace(void) {
    float res = 0.0f;
    const int dim = _rows < _cols ? _rows : _cols;
    for (int i = 0; i < dim; ++i) {
      res += (*this)[i][i];
    }
    return res;
  }

  float norm(void) { return std::sqrt((this->trans() * *this)[0][0]); }

 protected:
  int rows_, cols_;
  float data_[_rows * _cols];
};

namespace matrixf {

template <int _rows, int _cols>
Matrixf<_rows, _cols> zeros(void) {
  float data[_rows * _cols] = {0};
  return Matrixf<_rows, _cols>(data);
}

template <int _rows, int _cols>
Matrixf<_rows, _cols> ones(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < _rows * _cols; ++i) {
    data[i] = 1.0f;
  }
  return Matrixf<_rows, _cols>(data);
}

template <int _rows, int _cols>
Matrixf<_rows, _cols> eye(void) {
  float data[_rows * _cols] = {0};
  const int dim = _rows < _cols ? _rows : _cols;
  for (int i = 0; i < dim; ++i) {
    data[i * _cols + i] = 1.0f;
  }
  return Matrixf<_rows, _cols>(data);
}

template <int _rows, int _cols>
Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec) {
  Matrixf<_rows, _cols> res = matrixf::zeros<_rows, _cols>();
  const int dim = _rows < _cols ? _rows : _cols;
  for (int i = 0; i < dim; ++i) {
    res[i][i] = vec[i][0];
  }
  return res;
}

template <int _dim>
Matrixf<_dim, _dim> inv(Matrixf<_dim, _dim> mat) {
  Matrixf<_dim, 2 * _dim> ext_mat = matrixf::zeros<_dim, 2 * _dim>();
  for (int i = 0; i < _dim; ++i) {
    std::memcpy(ext_mat[i], mat[i], _dim * sizeof(float));
    ext_mat[i][_dim + i] = 1.0f;
  }

  for (int i = 0; i < _dim; ++i) {
    float abs_max = std::fabs(ext_mat[i][i]);
    int abs_max_row = i;
    for (int row = i + 1; row < _dim; ++row) {
      const float value = std::fabs(ext_mat[row][i]);
      if (abs_max < value) {
        abs_max = value;
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {
      return matrixf::zeros<_dim, _dim>();
    }
    if (abs_max_row != i) {
      Matrixf<1, 2 * _dim> row_i = ext_mat.row(i);
      Matrixf<1, 2 * _dim> row_abs_max = ext_mat.row(abs_max_row);
      std::memcpy(ext_mat[i], row_abs_max[0], 2 * _dim * sizeof(float));
      std::memcpy(ext_mat[abs_max_row], row_i[0],
                  2 * _dim * sizeof(float));
    }

    const float scale = 1.0f / ext_mat[i][i];
    for (int col = i; col < 2 * _dim; ++col) {
      ext_mat[i][col] *= scale;
    }
    for (int row = 0; row < _dim; ++row) {
      if (row == i) {
        continue;
      }
      const float factor = ext_mat[row][i];
      for (int col = i; col < 2 * _dim; ++col) {
        ext_mat[row][col] -= factor * ext_mat[i][col];
      }
    }
  }

  Matrixf<_dim, _dim> res;
  for (int i = 0; i < _dim; ++i) {
    std::memcpy(res[i], &ext_mat[i][_dim], _dim * sizeof(float));
  }
  return res;
}

}  // namespace matrixf

namespace vector3f {

Matrixf<3, 3> hat(Matrixf<3, 1> vec);
Matrixf<3, 1> cross(Matrixf<3, 1> vec1, Matrixf<3, 1> vec2);

}  // namespace vector3f

#endif  // MATRIX_PORTABLE_H
