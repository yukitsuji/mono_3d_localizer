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

#include "vertex_se3_quat.h"
#include "g2o/core/factory.h"

#include <iostream>
#include "g2o/core/cache.h"

using namespace Eigen;

namespace g2o {
namespace deprecated {

  VertexSE3::VertexSE3() :
    BaseVertex<6, SE3Quat>()
  {
    updateCache();
  }

  bool VertexSE3::read(std::istream& is)
  {

    Vector7D est;
    for (int i=0; i<7; i++)
      is  >> est[i];
    setEstimate(SE3Quat(est));
    updateCache();
    return true;
  }

  bool VertexSE3::write(std::ostream& os) const
  {
    for (int i=0; i<7; i++)
      os << estimate()[i] << " ";
    return os.good();
  }

  VertexSE3WriteGnuplotAction::VertexSE3WriteGnuplotAction(): WriteGnuplotAction(typeid(VertexSE3).name()){}

  HyperGraphElementAction* VertexSE3WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified" << std::endl;
      return nullptr;
    }
    
    VertexSE3* v =  static_cast<VertexSE3*>(element);
    *(params->os) << v->estimate().translation().x() << " " 
      << v->estimate().translation().y() << " " 
      << v->estimate().translation().z() << " ";
    *(params->os) << v->estimate().rotation().x() << " " 
      << v->estimate().rotation().y() << " " 
      << v->estimate().rotation().z() << " " << std::endl;
    return this;
  }


}
}
