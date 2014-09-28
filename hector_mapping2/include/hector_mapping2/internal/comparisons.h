//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#ifndef HECTOR_MAPPING2_INTERNAL_COMPARISONS_H
#define HECTOR_MAPPING2_INTERNAL_COMPARISONS_H

#include <hector_mapping2/types.h>

namespace hector_mapping
{
  static inline bool operator<(const GridIndex &index, const GridIndex &min)
  {
    return (index[0] < min[0]) || (index[1] < min[1]) || (index[2] < min[2]);
  }

  static inline bool operator>(const GridIndex &index, const GridIndex &max)
  {
    return (index[0] > max[0]) || (index[1] > max[1]) || (index[2] > max[2]);
  }

  static inline bool isValid(const GridIndex &index, const GridIndex &min, const GridIndex &max)
  {
    return !((index < min) || (index > max));
  }

  static inline GridIndex minGridIndex(const GridIndex &a, const GridIndex &b)
  {
    GridIndex min = a;
    if (b[0] < a[0]) min[0] = b[0];
    if (b[1] < a[1]) min[1] = b[1];
    if (b[2] < a[2]) min[2] = b[2];
    return min;
  }

  static inline GridIndex maxGridIndex(const GridIndex &a, const GridIndex &b)
  {
    GridIndex max = a;
    if (b[0] > a[0]) max[0] = b[0];
    if (b[1] > a[1]) max[1] = b[1];
    if (b[2] > a[2]) max[2] = b[2];
    return max;
  }

  static inline Point minPoint(const Point &a, const Point &b)
  {
    Point min = a;
    if (b[0] < a[0]) min[0] = b[0];
    if (b[1] < a[1]) min[1] = b[1];
    if (b[2] < a[2]) min[2] = b[2];
    return min;
  }

  static inline Point maxPoint(const Point &a, const Point &b)
  {
    Point max = a;
    if (b[0] > a[0]) max[0] = b[0];
    if (b[1] > a[1]) max[1] = b[1];
    if (b[2] > a[2]) max[2] = b[2];
    return max;
  }

} // namespace hector_mapping

#endif // HECTOR_MAPPING2_INTERNAL_COMPARISONS_H
