// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "vertex_ellipse.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"

#include <iomanip>

#include <Eigen/Eigenvalues>

using namespace std;

namespace g2o {

  VertexEllipse::VertexEllipse() : RobotData()
  {
    _covariance = Matrix3F::Zero();
    _UMatrix = Matrix2F::Zero();
    _singularValues = Vector2F::Zero();
  }

  VertexEllipse::~VertexEllipse()
  {
  }

  void VertexEllipse::_updateSVD() const{
    Eigen::SelfAdjointEigenSolver<Matrix2F> eigenSolver(_covariance.block<2,2>(0,0));
    _UMatrix = eigenSolver.eigenvectors();
    _singularValues = eigenSolver.eigenvalues();

  }

  bool VertexEllipse::read(std::istream& is)
  {
    float cxx, cxy, cxt, cyy, cyt, ctt;
    is >> cxx >> cxy >> cxt >> cyy >> cyt >> ctt;
    _covariance(0,0) = cxx;
    _covariance(0,1) = cxy;
    _covariance(0,2) = cxt;
    _covariance(1,0) = cxy;
    _covariance(1,1) = cyy;
    _covariance(1,2) = cyt;
    _covariance(2,0) = cxt;
    _covariance(2,1) = cyt;
    _covariance(2,2) = ctt;

    _updateSVD();

    int size;
    is >> size;
    for (int i =0; i<size; i++){
      float x, y;
      is >> x >> y;
      addMatchingVertex(x,y);
    }

    return true;
  }

  bool VertexEllipse::write(std::ostream& os) const
  {
    os << _covariance(0,0) << " " << _covariance(0,1) << " " << _covariance(0,2) << " "
       << _covariance(1,1) << " " << _covariance(1,2) << " " << _covariance(2,2) << " ";

    os << _matchingVertices.size() << " " ;
    for (size_t i=0 ; i< _matchingVertices.size(); i++){
      os << _matchingVertices[i].x() << " " << _matchingVertices[i].y() << " ";
    }

    return os.good();
  }

}
