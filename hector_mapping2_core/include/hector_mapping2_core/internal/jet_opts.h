/*
 * Copyright (c) 2012 libmv authors.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 *  deal in the Software without restriction, including without limitation the
 *  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef HECTOR_MAPPING2_CORE_JETOPS_H
#define HECTOR_MAPPING2_CORE_JETOPS_H

#ifdef HAVE_CERES
#include <ceres/ceres.h>

namespace ceres {

// A jet traits class to make it easier to work with mixed auto / numeric diff.
template<typename T>
struct JetOps {
  static bool IsScalar() {
    return true;
  }
  static T GetScalar(const T& t) {
    return t;
  }
  static void SetScalar(const T& scalar, T* t) {
    *t = scalar;
  }
  static void ScaleDerivative(double scale_by, T *value) {
    // For double, there is no derivative to scale.
  }
};

template<typename T, int N>
struct JetOps<Jet<T, N> > {
  static bool IsScalar() {
    return false;
  }
  static T GetScalar(const Jet<T, N>& t) {
    return t.a;
  }
  static void SetScalar(const T& scalar, Jet<T, N>* t) {
    t->a = scalar;
  }
  static void ScaleDerivative(double scale_by, Jet<T, N> *value) {
    value->v *= scale_by;
  }
};

}  // namespace ceres

#endif // HAVE_CERES
#endif // HECTOR_MAPPING2_CORE_JETOPS_H
