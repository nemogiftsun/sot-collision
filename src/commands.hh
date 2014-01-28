//
// Copyright 2010 CNRS
//
// Author: Florent Lamiraux
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// dynamic-graph is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with dynamic-graph.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SOT_COLLISION_COMMANDS_HH
#define SOT_COLLISION_COMMANDS_HH

#include "sot-collision/sotcollision/sot-collision.hh"




#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
using namespace std;
using namespace fcl;
namespace dynamicgraph {
  namespace sotcollision {
    namespace commands {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;
      class DoCapsuleCollisionCheck : public Command
      {
       
      public:
		virtual ~DoCapsuleCollisionCheck() {}
		/// Create a command and store it in Entity
		/// \param entity Instance of Entity owning of the command
		/// \param docstring documentation of the command
		DoCapsuleCollisionCheck(SotCollision& entity, const std::string& docstring):
		  Command(entity, boost::assign::list_of(Value::DOUBLE), docstring)
		{
		};


		virtual Value doExecute() 
		{
		  Entity& entity = owner();
		  SotCollision& ip = static_cast<SotCollision&>(entity);
		  std::vector<Value> values = getParameterValues();
	          ip.capsulecollision();
		  return Value();
		};
      }; //class addobject
class DoCapsuleBVHCollisionCheck : public Command
      {
       
      public:
		virtual ~DoCapsuleBVHCollisionCheck() {}
		/// Create a command and store it in Entity
		/// \param entity Instance of Entity owning of the command
		/// \param docstring documentation of the command
		DoCapsuleBVHCollisionCheck(SotCollision& entity, const std::string& docstring):
		  Command(entity, boost::assign::list_of(Value::DOUBLE), docstring)
		{
		};

		virtual Value doExecute() 
		{
		  Entity& entity = owner();
		  SotCollision& ip = static_cast<SotCollision&>(entity);
		  std::vector<Value> values = getParameterValues();
	          ip.capsulebvhcollision();
		  return Value();
		};
      }; //class addobject

class CreateLinkModel : public Command
      {


       
      public:
		virtual ~CreateLinkModel() {}
		/// Create a command and store it in Entity
		/// \param entity Instance of Entity owning of the command
		/// \param docstring documentation of the command
		CreateLinkModel(SotCollision& entity, const std::string& docstring):
		  Command(entity, boost::assign::list_of(Value::MATRIX), docstring)
		{
		};

		virtual Value doExecute() 
		{
		  Entity& entity = owner();
		  SotCollision& ip = static_cast<SotCollision&>(entity);
          std::vector<Value> values = getParameterValues();
          //double a = (values[0].value())(0,1);
          //Matrix a = values[0].value();
          //std::cout<< a;
	      ip.createfclmodel(values[0].value()); 
		  return Value();
		};
      }; //class addobject


class UpdateFCLModel : public Command
      {
       
      public:
		virtual ~UpdateFCLModel() {}
		/// Create a command and store it in Entity
		/// \param entity Instance of Entity owning of the command
		/// \param docstring documentation of the command
		UpdateFCLModel(SotCollision& entity, const std::string& docstring):
		  Command(entity, boost::assign::list_of(Value::DOUBLE), docstring)
		{
		};

		virtual Value doExecute() 
		{
		  Entity& entity = owner();
		  SotCollision& ip = static_cast<SotCollision&>(entity);
      int dummy;
	    ip.updatefclmodels(dummy,1);
		  return Value();
		};
      }; //class addobject



    } // namespace command
  } // namespace tutorial
} // namespace dynamicgraph
#endif //SOT_COLLISION_COMMANDS_HH

