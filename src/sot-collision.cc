/*
 *  Copyright 2013 CNRS
 *
 *  Nirmal Giftsun
 */



#include "sot-collision/sotcollision/sot-collision.hh"


#include "commands.hh"
#include "test_fcl_utility.h"


#include <jrl/mal/boost.hh>
#include "jrl/mal/matrixabstractlayer.hh"
namespace ml = maal::boost;

using namespace std;
using namespace fcl;
using namespace ml;
using namespace dynamicgraph::sotcollision;
using namespace dynamicgraph::sot;
using namespace ::dynamicgraph::command;
using namespace dynamicgraph;
using namespace collisiontest;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SotCollision, "SotCollision");

SotCollision::SotCollision(const std::string& inName) :
  Entity(inName),
  StatesOUT(boost::bind(&SotCollision::computeFCLOutput,this,_1,_2),sotNOSIGNAL,"SotCollision("+inName+")::output(matrix)::StatesOUT")
{
      size = 7;
      capsule_links.reserve(size);
      transform_links.reserve(size);
      transform_joint_links.reserve(size);
      //capsule_links = new Capsule[no_links];  

      signalRegistration(StatesOUT);
      
      
      

      //genericSignalRefs.push_back(fclLinkStatesOUT);
      
      


      using namespace ::dynamicgraph::command;
      std::string docstring;
      docstring = docCommandVoid2("Create a jacobian (world frame) signal only for one joint.",
				  "string (signal name)","string (joint name)");
      addCommand(std::string("DoCapsuleCollisionCheck"),
		 new commands::DoCapsuleCollisionCheck(*this, docstring));
      addCommand(std::string("DoCapsuleBVHCollisionCheck"),
		 new commands::DoCapsuleBVHCollisionCheck(*this, docstring));
  // getPendulumMass
  docstring =
    "\n"
    "    Get pendulum mass\n"
    "\n";
  addCommand(std::string("getcheck"),
	     new ::dynamicgraph::command::Getter<SotCollision, bool>
	     (*this, &SotCollision::getcheck, docstring));
  docstring =
    "\n"
    "    Get pendulum mass\n"
    "\n";
  addCommand(std::string("getdistance"),
	     new ::dynamicgraph::command::Getter<SotCollision, double>
	     (*this, &SotCollision::getdistance, docstring));	 
	 docstring =
    "\n"
    "    Get pendulum mass\n"
    "\n";
  addCommand(std::string("gettime"),
	     new ::dynamicgraph::command::Getter<SotCollision, double>
	     (*this, &SotCollision::gettime, docstring));	 	
  
    "\n"
    "    create model\n"
    "\n";
    addCommand(std::string("createlinkmodel"),
		     new commands::CreateLinkModel(*this, docstring));

    addCommand(std::string("getfclstate"),
	     new ::dynamicgraph::command::Getter<SotCollision, Matrix>
	     (*this, &SotCollision::getfclstate, docstring));	 	
  
  
 
    "\n"
    "    update fcl\n"
    "\n";
    addCommand(std::string("updatefclmodel"),
		     new commands::UpdateFCLModel(*this, docstring));
	 	 
/*

    
    "\n"
    "    Update capsular_links\n"
    "\n";
    addCommand(std::string("update_link_model"),
		     new commands::UpdateLinkModel(*this, docstring));	
docstring =
    "\n"
    "    Get fcl collision model info\n"
    "\n";
  addCommand(std::string("getcollisionmodelinfo"),
	     new ::dynamicgraph::command::Getter<SotCollision, std::vector<Tuple>>
	     (*this, &SotCollision::getcollisionmodelinfo, docstring));    
    */
}
 
		   


SotCollision::~SotCollision()
{
}
void SotCollision::capsulecollision()
{


        Timer timer;
    
        float v1,v2,x,y,z;
	    GJKSolver_libccd solver;
        int i = 0;
        //gettimeofday(&start, NULL);
        timer.start();
        
        while(i<1){
                
        v1 = 5; 
        v2 = 10 ;
        x= 23; 
        y = 0;
        z = 0; 

	Capsule capsulea (v1, v2);
	Transform3f capsulea_transform (Vec3f (x, y, z));

        v1 = 5; 
        v2 = 10;
        x = 0; 
        y = 0;
        z = 0; 

	Capsule capsuleb (v1, v2);
    Matrix3f rot(0.0,0,1,0,1,0,-1,0,0) ;
	Transform3f capsuleb_transform (rot,Vec3f (x, y, z));
	
	    FCL_REAL penetration = 0.;
	    FCL_REAL dist = 0.;
        Vec3f l1 , l2;
	    check = solver.shapeDistance<Capsule, Capsule>(capsulea, capsulea_transform, capsuleb, capsuleb_transform, &dist, &l1, &l2);
	    //check = solver.shapeIntersect(capsulea, capsulea_transform, capsuleb, capsuleb_transform, &contact_point, &penetration, &normal);
	    distance = dist;
        std::cout << l1;
        std::cout << l2; 
        //Transform3f temptransform (Vec3f (1, 1, 1));
        //Vec3f a = temptransform.transform(Vec3f (1, 2, 3));
        //std::cout << "vectranform" << std::endl;
        //std::cout << a[0];
        //std::cout << a[1];
        //std::cout << a[2];

        //if (i = 0)
        //{
          //gettimeofday(&start, NULL);
        //}
        i = i + 1;
        }
     
        //gettimeofday(&end, NULL);
        timer.stop();
        //diff = end.tv_usec - start.tv_usec - 15 ;
        diff = timer.getElapsedTimeInMicroSec() - 15;
   
        
}	  
void SotCollision::capsulebvhcollision()
{
    std::vector<TStruct> ts;

    ts = broad_phase_collision_test_customized(2000, 10000, 100, 1, false, true);

    TimeCheck = ts[1].overall_time;
}


dynamicgraph::SignalPtr< MatrixHomogeneous,int >& SotCollision::createPositionSignal( const std::string& signame)
{


  dynamicgraph::SignalPtr< MatrixHomogeneous,int> * sig
    = new dynamicgraph::SignalPtr< MatrixHomogeneous,int>
    (NULL,"sotCollision("+name+")::input(matrix)::"+signame);

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  return *sig;
  
}


Matrix& SotCollision::computeFCLOutput(Matrix& res, int t)
{
    int i;

    for(i=0;i<size;++i)
    {
       Vec3f translation = transform_links[i].getTranslation();
       Quaternion3f quat = transform_links[i].getQuatRotation();
        
       res(i,0) = translation[0];
       res(i,1) = translation[1]; 
       res(i,2) = translation[2];
       res(i,3) = quat[0];
       res(i,4) = quat[1];
       res(i,5) = quat[2];
       res(i,6) = quat[3];       
    }
    
    return res;
    
}


void SotCollision::UpdateFCLOutput()
{

    int i;

    for(i=0;i<size;++i)
    {
       

       fclstate.resize(size,9);

       Matrix T = *linkinputs[i];
       
       Matrix3f rot(T(0,0),T(0,1),T(0,2),T(1,0),T(1,1),T(1,2),T(2,0),T(2,1),T(2,2));
       Vec3f trans(T(0,3),T(1,3),T(2,3));
       
       transform_links[i]= Transform3f(rot,trans);
       transform_links[i] = transform_links[i] * transform_joint_links[i];
       
       Vec3f tpart = transform_links[i].getTranslation();
       Quaternion3f quat = transform_links[i].getQuatRotation();

       fclstate(i,0) = tpart[0];
       fclstate(i,1) = tpart[1]; 
       fclstate(i,2) = tpart[2];
      
       fclstate(i,3) = quat.getX();  
       fclstate(i,4) = quat.getY();
       fclstate(i,5) = quat.getZ();
       fclstate(i,6) = quat.getW(); 

       fclstate(i,7) = capsule_links[i].radius; 
       fclstate(i,8) = capsule_links[i].lz;     
  
    }
    

}

void SotCollision::createlinkmodel(const Matrix& linkdescription)
{
   
    Matrix3f rotation;
    int i;
 
    
    for(i=0;i<size;++i)
    {
        //std::cout << linkdescription(i,0);
        
        std::string signame_in = "link_" + boost::lexical_cast<std::string>(linkdescription(i,0));

        linkinputs[i] = &createPositionSignal(signame_in); 
        //create capsules for the links
        Capsule capsule (linkdescription(i,1),linkdescription(i,2));
        capsule_links.push_back(capsule);

        //create transforms
        Vec3f translation(linkdescription(i,3),linkdescription(i,4),linkdescription(i,5));
        rotation.setEulerZYX(linkdescription(i,6),linkdescription(i,7),linkdescription(i,8));
        //quatrotation.fromEuler(linkdescription(i,8),linkdescription(i,7),linkdescription(i,6));
        Transform3f joint_link_transform(rotation,translation);
        transform_joint_links.push_back(joint_link_transform);             
        transform_links.push_back(joint_link_transform);

        //std::string signame_out = "fcl_link_" + boost::lexical_cast<std::string>(linkdescription(i,0));
        //RH size = boost::get<1>(linkdescription[i]);  
        //XYZRPY pose= boost::get<2>(linkdescription[i]);


        
 
        //FCLlinkinputs[i] = &createFCLPositionSignal(signame_in); 
        //create input signals for joint 
       
       
    }

}


/*
void SotCollision::updatelinkmodel(transforms)
{
    Quaternion3f quatrotation;
    
    for(i=0;i<5;i++)
    {
        //get joint tranforms 
        Matrix3f rotation(transforms[i][0](0),transforms[i][0](1),transforms[i][0](2)
                          ,transforms[i][1](0),transforms[i][1](1),transforms[i][1](2)
                          ,transforms[i][2](0),transforms[i][0](1),transforms[i][0](2));
        Vec3f translation(transforms[i][0](3),transforms[i][1](3),transforms[i][2](3));
       
        //update transforms
        transform_links[i].setTransform(rotation,translation);

    }

}


std::tuple SotCollision::getcollisionmodelinfo()
{
    
     std::make_tuple();
     capsule_links
     transform_links
     
     std::tuple a;
     return a;

}*/

