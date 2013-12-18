/*
 * Copyright 2013,
 * Nirmal Giftsun
 *
 * CNRS
 *
 * This file is part of sot-collision.
 * dynamic-graph-tutorial is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * dynamic-graph-tutorial is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with dynamic-graph-tutorial.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef COLLISION_MODEL_HH
# define COLLISION_MODEL_HH

// sot includes
#include <sot-dynamic/dynamic.h>


// dynamic graph includes
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>

// fcl includes
#include "fcl/articulated_model/link.h"
#include "fcl/articulated_model/joint.h"
#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"


#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

// specific includes
#include "../../../src/collision_test.hh"



using namespace fcl;
namespace dynamicgraph {
  namespace sotcollision {

class CollisionModel : public Entity
    {
    public:
      /**
	 \brief Constructor by name
      */
      SotCollision(const std::string& inName);

      virtual ~SotCollision(void);

      /// Each entity should provide the name of the class it belongs to
      virtual const std::string& getClassName (void) const {
	return CLASS_NAME;
      }

      /// Header documentation of the python class
      virtual std::string getDocString () const {
	return "sot-collision\n";
      }

      /**
	  \name Parameters
	  @{
      */
      double getcheck () const {
	return TimeCheck;
      }
      
      void capsulecollision();

      void capsulebvhcollision();

    protected:
      /*
	\brief Class name
      */
      static const std::string CLASS_NAME;

    private:
      bool check;
      int diff;
      double TimeCheck;
      /**
	 \brief input trigger
      */
      SignalPtr< double, int > triggerSIN;
      /**
	 \brief state
      */
      Signal< ::dynamicgraph::Vector, int> stateSOUT;

    };
  }
}

#endif
