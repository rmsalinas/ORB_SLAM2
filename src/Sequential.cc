#include "Sequential.h"
#include <iostream>
using namespace  std;
namespace ORB_SLAM2 {
Sequential::Queue<int> Sequential::_LocalOptQ;
Sequential::Queue<int> Sequential::_LoopClosingQ;
bool Sequential::_playNFFlag=true;
bool Sequential::_enabled=false;
void Sequential::localMappingClear(){
       if(_enabled) {
           cout<<"|||||| Sequential::localMappingClear"<<endl;
           _LocalOptQ.clear();
       }
}
  void Sequential::waitForEndLocalMapping(){
  if(_enabled) {
      cout<<"|||||| Sequential::waitForEndLocalMapping"<<endl;
      _LocalOptQ.pop();
  }
}
  void Sequential::endLocalMapping(){
    if(_enabled) {
        cout<<"|||||| Sequential::endLocalMapping"<<endl;
        _LocalOptQ.push(1);
    }
}
  void Sequential::loopClosingClear(){

     if(_enabled) {
               cout<<"|||||| Sequential::loopClosingClear"<<endl;
               _LoopClosingQ.clear();
     }
  }
  void Sequential::endLoopClosing(){

    if(_enabled){
        cout<<"|||||| Sequential::endLoopClosing"<<endl;
        _LoopClosingQ.push(1);
    }
}
  void Sequential::waitForEndLoopClosing(){

    if(_enabled) {
        cout<<"|||||| Sequential::waitForEndLoopClosing"<<endl;
        _LoopClosingQ.pop();
    }
}

}
