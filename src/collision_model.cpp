createPositionSignal( const std::string& signame, CjrlJoint* aJoint)
{
  sotDEBUGIN(15);

  dg::SignalTimeDependent< MatrixHomogeneous,int > * sig
    = new dg::SignalTimeDependent< MatrixHomogeneous,int >
    ( boost::bind(&Dynamic::computeGenericPosition,this,aJoint,_1,_2),
      newtonEulerSINTERN,
      "sotDynamic("+name+")::output(matrixHomo)::"+signame );

  genericSignalRefs.push_back( sig );
  signalRegistration( *sig );

  sotDEBUGOUT(15);
  return *sig;
}
