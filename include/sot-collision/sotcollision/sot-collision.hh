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



using namespace std;
using namespace fcl;
using namespace collisiontest;
using namespace ml;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

//point in local coordinate frame
typedef boost::tuples::tuple<FCL_REAL,FCL_REAL,FCL_REAL> XYZ;

// tuple of inter-model distance and the closest point to the other model
typedef boost::tuples::tuple<FCL_REAL,XYZ> One2OneModelDistance;

// vector of model information
typedef std::vector<One2OneModelDistance> One2ManyModelDistance;

// vector of model information
typedef std::vector<One2ManyModelDistance> InterModelDistanceMatrix;



namespace dynamicgraph {
  namespace sotcollision {

struct link_info {string link_type; string link_location;int index;};

bool operator < (const link_info &link1,const link_info  &link2 ) { return 0; }

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

      // methods
      void experimentcollision();

      int& updatefclmodels(int& dummy,int time);

      void createcollisionpair(const std::string& body0, const std::string& body1);

      void createcollisionlink(const std::string& bodyname,const std::string& bodytype,const std::string& bodynaturelocation, const Vector& bodydescription);

      Vector& computeimdVector(Vector& res, int time );

      Matrix& computeimdJacobian(Matrix& res,int time );


      // vars
      dynamicgraph::SignalPtr< MatrixHomogeneous,int >& createPositionSignalIN(const std::string& signame);

      dynamicgraph::SignalPtr< Matrix,int >& createJacobiansignalIN(const std::string& signame);

      dynamicgraph::SignalTimeDependent< Matrix,int >& createIMDJacobianSignal();

      dynamicgraph::SignalTimeDependent< Vector,int >& createInterModelDistanceSignal();

      std::map<std::string,int> fcl_body_map; 

      std::map<int,link_info> link_info_map; 

      MatrixHomogeneous UNIT_ROTATION;

    protected:
      /*
	\brief Class name
      */
      static const std::string CLASS_NAME;

    private:
      int dimension;
      //intern signals
      dynamicgraph::SignalTimeDependent<int,int> fclmodelupdateSINTERN;    
      dynamicgraph::SignalPtr< MatrixHomogeneous,int > *body_transformation_input[10];
      dynamicgraph::SignalPtr< Matrix,int > *body_jacobian_input[10];
      dynamicgraph::SignalPtr< Matrix,int > *collision_body_jacobian_input[10];
      dynamicgraph::SignalTimeDependent<ml::Vector,int> InterModelDistanceOUT;
      dynamicgraph::SignalTimeDependent< Vector,int > collisionDistance;
      dynamicgraph::SignalTimeDependent<Matrix,int >collisionJacobian;


      std::vector<Capsule> capsule_links;
      std::vector<Box> box_links;
      std::vector<Transform3f> transform_links;
      std::vector<Transform3f> transform_joint_links;
      std::list< dynamicgraph::SignalBase<int>*  > genericSignalRefs;
      bool check;
      double diff;
      double distance;
      double TimeCheck;
      Matrix fclstate;
      Matrix LinkDescription;
      // collision pairs
      std::vector< std::vector<std::string> >  collision_pairs;
      int num_collisionpairs;
      InterModelDistanceMatrix imdm;

    };
  }
}

#endif
/*
dynamicgraph::SignalTimeDependent<Matrix,int>& SotCollision::createIMDJacobianSignal( )
{
  dynamicgraph::SignalTimeDependent<Matrix,int > * sig
    = new dynamicgraph::SignalTimeDependent<Matrix,int>
    ( boost::bind(&SotCollision::computeimdJacobian,this,_1,_2),
      fclmodelupdateSINTERN,
      "sotCollision("+name+")::output(matrix)::"+std::string("collisionJacobian"));

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sig->addDependency(fclmodelupdateSINTERN);
  return *sig;
}
dynamicgraph::SignalTimeDependent<Vector,int>& SotCollision::createInterModelDistanceSignal( )
{

  dynamicgraph::SignalTimeDependent< Vector,int > * sig
    = new dynamicgraph::SignalTimeDependent<Vector,int>
    ( boost::bind(&SotCollision::computeimdVector,this,_1,_2),
      fclmodelupdateSINTERN,
      "sotCollision("+name+")::output(vector)::"+std::string("collisionDistance"));

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );
   sig->addDependency(fclmodelupdateSINTERN);


  return *sig;  
}*/

