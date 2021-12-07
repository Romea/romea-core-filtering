//#ifndef romea_BayesianFilterUpdater_hpp
//#define romea_BayesianFilterUpdater_hpp

////std
//#include <atomic>
//#include <fstream>
//#include <memory>

////romea
//#include <FilterMetaState.hpp>

//namespace romea {


//template < class State , class Observation , class FSMState, class Duration>
//class FilterUpdater
//{

//public :

//  using UpdateFunction =  std::function<void(const Duration &, FSMState &, State&)> ;

//public :

//  FilterUpdater();

//  virtual ~FilterUpdater()=default;

//  virtual void update(const Duration & duration,
//                      const Observation & currentObservation,
//                      FSMState & currentStatus,
//                      State & currentstate) =0;


//  UpdateFunction makeUpdateFunction(Observation && observation);

//};


////-----------------------------------------------------------------------------
//template < class State , class Observation , class FSMState, class Duration>
//std::function<void(const Duration &, FSMState &, State&)>
//FilterUpdater<State,Observation,FSMState,Duration>::makeUpdateFunction(Observation && observation)
//{
//  return std::bind(&FilterUpdater<State,Observation,FSMState,Duration>::update,
//                   this,
//                   std::placeholders::_1,
//                   std::move(observation),
//                   std::placeholders::_2,
//                   std::placeholders::_3);
//}


//}//romea

//#endif
