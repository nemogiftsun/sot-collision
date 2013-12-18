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

#ifndef DG_SOT_COLLISION_HH
#define DG_SOT_COLLISION_HH

#include <boost/format.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>



#include <jrl/mal/boost.hh>
#include "jrl/mal/matrixabstractlayer.hh"
namespace ml = maal::boost;


/* FCL */
#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

/* SOT */
#include <sot/core/flags.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/exception-dynamic.hh>
#include <sot/core/matrix-homogeneous.hh>


#include "../../../src/collision_test.hh"




using namespace fcl;
using namespace collisiontest;
using namespace ml;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;


typedef boost::tuples::tuple<FCL_REAL,FCL_REAL,FCL_REAL,FCL_REAL,FCL_REAL,FCL_REAL> XYZRPY;

typedef boost::tuples::tuple<FCL_REAL,FCL_REAL> RH;

typedef boost::tuples::tuple<std::string,RH,XYZRPY> Link_Description;


namespace dynamicgraph {
  namespace sotcollision {

class SotCollision : public Entity
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
      bool getcheck () const {
	return check;
      }
      
            double getdistance () const {
	return distance;
      }
      
                  double gettime () const {
	return diff;
      }

                  Matrix getfclstate () const {
	return fclstate;
      }
      void capsulecollision();

      void capsulebvhcollision();

      void UpdateFCLOutput();

      void createlinkmodel(const Matrix& linkdescription);

      dynamicgraph::SignalPtr< MatrixHomogeneous,int >& createPositionSignal(const std::string& signame);



      Matrix& computeFCLOutput(Matrix& res, int t);



    protected:
      /*
	\brief Class name
      */
      static const std::string CLASS_NAME;

    private:

      int size;
      dynamicgraph::SignalTimeDependent< Matrix,int> StatesOUT;
      dynamicgraph::SignalPtr< MatrixHomogeneous,int > *linkinputs[10];
      std::vector<Capsule> capsule_links;
      std::vector<Transform3f> transform_links;
      std::vector<Transform3f> transform_joint_links;
      std::list< dynamicgraph::SignalBase<int>*  > genericSignalRefs;
      bool check;
      double diff;
      double distance;
      double TimeCheck;
      Matrix fclstate;
      Matrix LinkDescription;

    };
  }
}

#endif
